function getQueryParameters() {
  var params = {};
  window.location.search
    .substr(1)
    .split("&")
    .forEach(function (p) {
      var param = p.split("=");
      params[param[0]] = decodeURIComponent(param[1]);
    });
  return params;
}

function getBackendUrl(params) {
  const apiPath = "api/";
  let apiEndpoint = window.location.origin + window.location.pathname;
  if (!apiEndpoint.endsWith("/")) {
    apiEndpoint += "/";
  }
  apiEndpoint += apiPath;

  const backendParam = params["backend"] || null;
  if (backendParam) {
    if (/^[0-9]+$/.test(backendParam)) {
      apiEndpoint =
        "//" + window.location.hostname + ":" + backendParam + "/" + apiPath;
    } else if (
      !backendParam.startsWith("http://") &&
      !backendParam.startsWith("https://") &&
      !backendParam.startsWith("//")
    ) {
      apiEndpoint = "//" + backendParam;
    } else {
      apiEndpoint = backendParam;
    }
    if (!apiEndpoint.endsWith("/")) {
      apiEndpoint += "/";
    }
  }
  return apiEndpoint;
}

var params = getQueryParameters();
var backend = getBackendUrl(params);

var map = L.map("map", {
  contextmenu: true,
  contextmenuWidth: 200,
  contextmenuItems: [
    {
      text: "Set start",
      callback: setRoutingStart,
    },
    {
      text: "Set destination",
      callback: setRoutingDest,
    },
    {
      text: "Reset routing request",
      callback: clearRoutingRequest,
    },
    {
      separator: true,
    },
    {
      text: "Center map here",
      callback: function (e) {
        map.panTo(e.latlng);
      },
    },
    {
      text: "Zoom in here",
      callback: function (e) {
        map.setView(e.latlng, 20);
      },
    },
    {
      text: "Show coordinates",
      callback: function (e) {
        window.prompt(
          "Coordinates:",
          e.latlng.lat.toFixed(6) + ";" + e.latlng.lng.toFixed(6)
        );
      },
    },
    {
      separator: true,
    },
    {
      text: "Load routing graph overlay",
      callback: requestGeoJSONOverlay,
    },
  ],
});
map.setView([49.8728, 8.6512], 14);

L.tileLayer("//{s}.tile.osm.org/{z}/{x}/{y}.png", {
  attribution:
    '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
  maxZoom: 22,
  maxNativeZoom: 19,
}).addTo(map);

L.control
  .scale({
    maxWidth: 240,
    metric: true,
    imperial: false,
    position: "bottomleft",
  })
  .addTo(map);

L.control.polylineMeasure({ showMeasurementsClearControl: true }).addTo(map);

var ppr = L.Routing.ppr({
  serviceUrl: backend,
});

var routingControl = L.Routing.control({
  waypoints: [
    // L.latLng(49.8775462288535, 8.656797409057619),
    // L.latLng(49.87023075540512, 8.651347160339357)
  ],
  language: "en",
  router: ppr,
  plan: L.Routing.plan([], {
    geocoder: L.Control.Geocoder.latLng({
      next: L.Control.Geocoder.nominatim({
        serviceUrl: "https://nominatim.openstreetmap.org/",
      }),
    }),
    waypointNameFallback: function (latLng) {
      var lat = (
          Math.round(Math.abs(latLng.lat) * 1000000) / 1000000
        ).toString(),
        lng = (Math.round(Math.abs(latLng.lng) * 1000000) / 1000000).toString();
      return lat + ", " + lng;
    },
    maxGeocoderTolerance: 50,
  }),
  routeWhileDragging: true,
  routeDragInterval: 50,
  reverseWaypoints: true,
  showAlternatives: true,
  lineOptions: {
    styles: [
      { color: "black", opacity: 0.15, weight: 9 },
      { color: "white", opacity: 0.8, weight: 6 },
      { color: "red", opacity: 1, weight: 2 },
    ],
  },
  altLineOptions: {
    styles: [
      { color: "black", opacity: 0.15, weight: 9 },
      { color: "white", opacity: 0.8, weight: 6 },
      { color: "blue", opacity: 0.5, weight: 2 },
    ],
  },
  collapsible: true,
}).addTo(map);

L.Routing.errorControl(routingControl).addTo(map);

var geoJSONOverlay = L.control.geoJSONOverlay({ backend: backend }).addTo(map);

L.control.mapLinks().addTo(map);

function setRoutingStart(e) {
  routingControl.spliceWaypoints(0, 1, e.latlng);
}

function setRoutingDest(e) {
  routingControl.spliceWaypoints(
    routingControl.getWaypoints().length - 1,
    1,
    e.latlng
  );
}

function clearRoutingRequest(e) {
  routingControl.setWaypoints([]);
}

function requestGeoJSONOverlay(e) {
  geoJSONOverlay._requestData();
}

function updateUrlHash() {
  var center = map.getCenter();
  var hash = "#" + center.lat + ";" + center.lng + ";" + map.getZoom();
  history.replaceState({}, "", hash);
}

function handleUrlHash() {
  var hash = window.location.hash.substring(1);
  var values = hash.split(";").map(function (s) {
    return parseFloat(s);
  });
  if (values.length >= 2) {
    var lat = values[0];
    var lng = values[1];
    var zoom = values[2];
    map.setView(L.latLng(lat, lng), zoom);
    //geoJSONOverlay._setAutoRefresh(true);
  }
}

map.on({ moveend: updateUrlHash, zoomend: updateUrlHash });
window.addEventListener("hashchange", handleUrlHash);
window.addEventListener("load", handleUrlHash);

window.addEventListener("load", function () {
  var container = document.querySelector(".leaflet-routing-geocoders");

  var profileSelectorContainer = L.DomUtil.create(
    "div",
    "profile-selector",
    container
  );

  var select = L.DomUtil.create("select", "", profileSelectorContainer);
  for (var p of searchProfiles) {
    var opt = document.createElement("option");
    opt.innerText = p;
    select.appendChild(opt);
  }

  L.DomEvent.on(select, "change", function (e) {
    ppr.options.searchProfile = e.target.value;
    routingControl.route();
  });
});
