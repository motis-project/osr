L.Control.GeoJSONOverlay = L.Control.extend({
  options: {
    position: "topleft",
    backend: "",
    backgroundColor: "#8f8",
  },

  onAdd: function (map) {
    var self = this;
    self._map = map;
    self._container = document.createElement("div");
    self._container.classList.add("leaflet-bar");
    L.DomEvent.disableClickPropagation(self._container);

    self._toggleAutoRefreshButton = document.createElement("a");
    self._toggleAutoRefreshButton.innerHTML = "G";
    self._toggleAutoRefreshButton.title = "Show routing graph overlay";
    L.DomEvent.on(
      self._toggleAutoRefreshButton,
      "click",
      self._toggleAutoRefresh,
      self
    );
    self._container.appendChild(self._toggleAutoRefreshButton);

    self._toggleAreasButton = document.createElement("a");
    self._toggleAreasButton.innerHTML = "A";
    self._toggleAreasButton.title = "Show areas in routing graph overlay";
    L.DomEvent.on(self._toggleAreasButton, "click", self._toggleAreas, self);
    self._container.appendChild(self._toggleAreasButton);

    self._toggleVisibilityGraphButton = document.createElement("a");
    self._toggleVisibilityGraphButton.innerHTML = "VG";
    self._toggleVisibilityGraphButton.title =
      "Show visibility graph in routing graph overlay";
    L.DomEvent.on(
      self._toggleVisibilityGraphButton,
      "click",
      self._toggleVisibilityGraphs,
      self
    );
    self._container.appendChild(self._toggleVisibilityGraphButton);

    self._overlay = null;
    self._autoRefresh = false;
    self._includeAreas = false;
    self._includeVisibilityGraphs = false;

    var refresh = L.Util.throttle(self._refreshOverlay, 1000, self);
    map.on({
      moveend: refresh,
      resize: refresh,
      zoomend: refresh,
    });

    return self._container;
  },

  _refreshOverlay: function () {
    if (this._autoRefresh) {
      this._requestData();
    }
  },

  _requestData: function () {
    var self = this;

    if (self._map.getZoom() < 17) {
      if (
        self._autoRefresh ||
        !window.confirm(
          "Are you sure you want to load the routing graph overlay for zoom level " +
            self._map.getZoom() +
            "?"
        )
      ) {
        return;
      }
    }

    var url = self.options.backend + "graph";

    var headers = new Headers();
    headers.append("Content-Type", "application/json");

    var bounds = self._map.getBounds();
    var request = {
      waypoints: [
        bounds.getWest(),
        bounds.getNorth(),
        bounds.getEast(),
        bounds.getSouth(),
      ],
      include_areas: self._includeAreas,
      include_visibility_graphs: self._includeVisibilityGraphs,
    };

    window
      .fetch(url, {
        method: "POST",
        body: JSON.stringify(request),
        headers: headers,
        mode: "cors",
      })
      .then(function (response) {
        return response.json();
      })
      .then(function (response) {
        self._removeOverlay();
        self._overlay = L.geoJSON(response, {
          style: self._style,
          onEachFeature: self._onEachFeature.bind(self),
          pointToLayer: self._pointToLayer.bind(self),
        }).addTo(map);
      });
  },

  _toggleAutoRefresh: function () {
    var self = this;
    self._setAutoRefresh(!self._autoRefresh);
  },

  _setAutoRefresh: function (enabled) {
    var self = this;
    if (self._autoRefresh != enabled) {
      self._autoRefresh = enabled;
      if (self._autoRefresh) {
        self._toggleAutoRefreshButton.style.backgroundColor =
          self.options.backgroundColor;
        self._refreshOverlay();
      } else {
        self._toggleAutoRefreshButton.removeAttribute("style");
        self._hideOverlay();
      }
    }
  },

  _toggleAreas: function () {
    var self = this;
    self._setIncludeAreas(!self._includeAreas);
  },

  _setIncludeAreas: function (enabled, noRefresh) {
    var self = this;
    if (self._includeAreas != enabled) {
      self._includeAreas = enabled;
      if (self._includeAreas) {
        self._toggleAreasButton.style.backgroundColor =
          self.options.backgroundColor;
      } else {
        self._toggleAreasButton.removeAttribute("style");
      }
      if (self._autoRefresh && noRefresh !== true) {
        self._refreshOverlay();
      }
    }
  },

  _toggleVisibilityGraphs: function () {
    var self = this;
    self._setIncludeVisibilityGraphs(!self._includeVisibilityGraphs);
  },

  _setIncludeVisibilityGraphs: function (enabled) {
    var self = this;
    if (self._includeVisibilityGraphs != enabled) {
      self._includeVisibilityGraphs = enabled;
      if (self._includeVisibilityGraphs) {
        self._toggleVisibilityGraphButton.style.backgroundColor =
          self.options.backgroundColor;
        self._setIncludeAreas(true, true);
      } else {
        self._toggleVisibilityGraphButton.removeAttribute("style");
      }
      if (self._autoRefresh) {
        self._refreshOverlay();
      }
    }
  },

  _hideOverlay: function () {
    var self = this;

    self._setAutoRefresh(false);
    self._removeOverlay();
  },

  _removeOverlay: function () {
    var self = this;

    if (self._overlay) {
      self._overlay.remove();
      self._overlay = null;
    }
  },

  _style: function (feature) {
    var color = undefined;
    var fillOpacity = undefined;
    if (feature.style) {
      color = feature.style.stroke;
      fillOpacity = 0.2;
    }
    return {
      color: color,
      fillOpacity: fillOpacity,
      weight: 3,
    };
  },

  _onEachFeature: function (feature, layer) {
    var self = this;
    if (feature.properties === undefined) {
      return;
    }
    layer.bindPopup(function (l) {
      var table = document.createElement("table");
      table.classList.add("routing-graph", "properties");
      for (var key in feature.properties) {
        var value = feature.properties[key];
        var row = document.createElement("tr");
        var keyCell = document.createElement("td");
        keyCell.innerText = key;
        keyCell.className = "key";
        var valueCell = document.createElement("td");
        if (key == "osm_way_id" && value != 0) {
          var osmLink = document.createElement("a");
          osmLink.href = "https://www.openstreetmap.org/way/" + Math.abs(value);
          osmLink.target = "_blank";
          osmLink.innerText = value;
          valueCell.appendChild(osmLink);
        } else if (key == "osm_relation_id" && value != 0) {
          var osmLink = document.createElement("a");
          osmLink.href =
            "https://www.openstreetmap.org/relation/" + Math.abs(value);
          osmLink.target = "_blank";
          osmLink.innerText = value;
          valueCell.appendChild(osmLink);
        } else if (key == "osm_node_id" && value != 0) {
          var osmLink = document.createElement("a");
          osmLink.href =
            "https://www.openstreetmap.org/node/" + Math.abs(value);
          osmLink.target = "_blank";
          osmLink.innerText = value;
          valueCell.appendChild(osmLink);
        } else {
          valueCell.innerText = value;
        }
        valueCell.className = "value";
        row.appendChild(keyCell);
        row.appendChild(valueCell);
        table.appendChild(row);
      }
      return table;
    });
    layer.on(
      {
        mouseover: self._highlightFeature,
        mouseout: self._resetHighlight,
      },
      self
    );
  },

  _pointToLayer: function (feature, latlng) {
    return L.circle(latlng, {
      radius: 0.5,
      fillColor: "#C1120E",
      color: "#000",
      weight: 1,
      opacity: 1,
      fillOpacity: 0.8,
    });
  },

  _highlightFeature: function (e) {
    var layer = e.target;

    layer.setStyle({
      weight: 5,
    });

    if (!L.Browser.ie && !L.Browser.opera && !L.Browser.edge) {
      layer.bringToFront();
    }
  },

  _resetHighlight: function (e) {
    var self = this;
    self._overlay.resetStyle(e.target);
  },
});

L.control.geoJSONOverlay = function (opts) {
  return new L.Control.GeoJSONOverlay(opts);
};
