L.Control.MapLinks = L.Control.extend({
  options: {
    position: "topleft",
  },

  onAdd: function (map) {
    var self = this;
    self._map = map;
    self._container = document.createElement("div");
    self._container.classList.add("leaflet-bar");
    L.DomEvent.disableClickPropagation(self._container);

    self._container.appendChild(
      self._createButton("O", "OpenStreetMap", function () {
        var center = this._map.getCenter();
        var zoom = Math.round(this._map.getZoom());
        var url =
          "https://www.openstreetmap.org/#map=" +
          zoom +
          "/" +
          center.lat.toFixed(5) +
          "/" +
          center.lng.toFixed(5) +
          "&layers=D";
        window.open(url);
      })
    );

    self._container.appendChild(
      self._createButton("G", "Google Maps", function () {
        var center = this._map.getCenter();
        var zoom = Math.round(this._map.getZoom());
        var url =
          "https://www.google.com/maps" +
          L.Util.getParamString({
            ll: center.lat.toFixed(6) + "," + center.lng.toFixed(6),
            z: zoom,
            t: "h",
          });
        window.open(url);
      })
    );

    self._container.appendChild(
      self._createButton("MC", "Map Compare", function () {
        var center = this._map.getCenter();
        var zoom = Math.round(this._map.getZoom());
        var url =
          "http://tools.geofabrik.de/mc/#" +
          zoom +
          "/" +
          center.lat.toFixed(4) +
          "/" +
          center.lng.toFixed(4) +
          "&num=4&mt0=mapnik&mt1=google-map&mt2=google-hybrid&mt3=here-hybrid";
        window.open(url);
      })
    );

    self._container.appendChild(
      self._createButton("M", "Mapillary", function () {
        var center = this._map.getCenter();
        var zoom = Math.round(this._map.getZoom());
        var url =
          "https://www.mapillary.com/app/" +
          L.Util.getParamString({
            lat: center.lat,
            lng: center.lng,
            z: zoom,
          });
        window.open(url);
      })
    );

    self._container.appendChild(
      self._createButton("OT", "Overpass Turbo", function () {
        var center = this._map.getCenter();
        var zoom = Math.round(this._map.getZoom());
        var query = window.prompt(
          "Overpass Turbo query (wizard syntax, e.g. highway=primary or highway=*):",
          ""
        );
        if (query === null) {
          return;
        }
        var url =
          "http://overpass-turbo.eu/?C=" +
          encodeURIComponent(center.lat + ";" + center.lng + ";" + zoom);
        if (query != "") {
          url += "&w=" + encodeURIComponent(query) + "&R=1";
        }
        window.open(url);
      })
    );

    self._container.appendChild(
      self._createButton("Ex", "Osmium Extract", function () {
        var bounds = this._map.getBounds();
        var b =
          "-b " +
          bounds.getWest().toFixed(6) +
          "," +
          bounds.getSouth().toFixed(6) +
          "," +
          bounds.getEast().toFixed(6) +
          "," +
          bounds.getNorth().toFixed(6);
        window.prompt(
          "osmium extract --set-bounds [-b ...] input.osm.pbf -o output.osm.pbf --overwrite",
          b
        );
      })
    );

    self._container.appendChild(
      self._createButton("J", "JOSM", function () {
        // https://josm.openstreetmap.de/wiki/Help/Preferences/RemoteControl
        var bounds = this._map.getBounds();
        var url =
          "http://127.0.0.1:8111/load_and_zoom" +
          L.Util.getParamString({
            left: bounds.getWest(),
            right: bounds.getEast(),
            top: bounds.getNorth(),
            bottom: bounds.getSouth(),
          });
        window.fetch(url).then(
          function (response) {
            if (!response.ok) {
              alert("JOSM Error: " + response.statusText);
            }
          },
          function (reason) {
            alert(
              "Connection to JOSM failed (" +
                reason +
                "). Please make sure that JOSM is running and that remote control is enabled."
            );
            window.open(
              "https://josm.openstreetmap.de/wiki/Help/Preferences/RemoteControl"
            );
          }
        );
      })
    );

    return self._container;
  },

  _createButton: function (label, title, onclick) {
    var self = this;
    var button = document.createElement("a");
    button.innerHTML = label;
    button.title = title;
    L.DomEvent.on(button, "click", onclick, self);
    return button;
  },
});

L.control.mapLinks = function (opts) {
  return new L.Control.MapLinks(opts);
};
