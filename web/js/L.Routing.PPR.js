L.Routing.PPR = L.Class.extend({
  options: {
    serviceUrl:
      window.location.origin +
      (window.location.origin.endsWith("/") ? "api/" : "/api/"),
    timeout: 30 * 1000,
    searchProfile: 'foot',
  },

  initialize: function (options) {
    L.Util.setOptions(this, options);
  },

  route: function (waypoints, callback, context, options) {
    var timedOut = false,
      wps = [],
      url,
      request,
      timer,
      wp,
      i,
      self = this;

    url = this.options.serviceUrl + "route";
    request = this.buildRouteRequest(
      waypoints,
      options,
      self.options.searchProfile
    );
    if (this.options.requestParameters) {
      url += L.Util.getParamString(this.options.requestParameters, url);
    }

    timer = setTimeout(function () {
      timedOut = true;
      callback.call(context || callback, {
        status: -1,
        message: "Routing request timed out.",
      });
    }, this.options.timeout);

    // Create a copy of the waypoints, since they
    // might otherwise be asynchronously modified while
    // the request is being processed.
    for (i = 0; i < waypoints.length; i++) {
      wp = waypoints[i];
      wps.push(new L.Routing.Waypoint(wp.latLng, wp.name, wp.options));
    }

    var headers = new Headers();
    headers.append("Content-Type", "application/json");

    window
      .fetch(url, {
        method: "POST",
        body: JSON.stringify(request),
        headers: headers,
        mode: "cors",
      })
      .then(
        function (response) {
          return response.json();
        },
        function (reason) {
          console.log("routing: fetch failed:", reason);
          callback.call(context || callback, {
            status: -1,
            message: "HTTP request failed: " + reason,
          });
        }
      )
      .then(
        function (response) {
          clearTimeout(timer);
          if (!timedOut) {
            try {
              self._routeDone(response, wps, options, callback, context);
            } catch (ex) {
              callback.call(context || callback, {
                status: -3,
                message: ex.toString(),
              });
            }
          }
        },
        function (reason) {
          console.log("routing: parse failed:", reason);
          callback.call(context || callback, {
            status: -2,
            message: "Error parsing routing response: " + reason,
          });
        }
      );
  },

  _routeDone: function (response, inputWaypoints, options, callback, context) {
    context = context || callback;
    if (response.error) {
      callback.call(context, {
        status: 1,
        message: response.error,
      });
      return;
    }

    var routes = response.routes.map(function (responseRoute) {
      var route = this._convertRoute(responseRoute);
      route.inputWaypoints = inputWaypoints;
      return route;
    }, this);

    if (routes.length === 0) {
      callback.call(context, {
        status: 1,
        message: "No routes found.",
      });
      return;
    }

    callback.call(context, null, routes);
  },

  _convertRoute: function (responseRoute) {
    const coordinates = responseRoute.path.map(function (c) {
      return L.latLng(c[1], c[0]);
    });

    const getStepText = function (s) {
      switch (s.step_type) {
        case "street":
          return s.street_name == "" ? "Street" : s.street_name;
        case "footway":
          switch (s.street_type) {
            case "stairs":
              return "Stairs";
            case "escalator":
              return "Escalator";
            case "moving_walkway":
              return "Moving walkway";
            default:
              return s.street_name == "" ? "Footpath" : s.street_name;
          }
        case "crossing": {
          let text = "Cross ";
          if (s.street_name == "") {
            switch (s.street_type) {
              case "rail":
                text += "the train tracks";
                break;
              case "tram":
                text += "the tram tracks";
                break;
              default:
                text += "the street";
                break;
            }
          } else {
            text += s.street_name;
          }
          switch (s.crossing_type) {
            case "marked":
              text += " (at the crosswalk)";
              break;
            case "signals":
              text += " (at the traffic signals)";
              break;
            case "islands":
              text += " (using the traffic island)";
              break;
          }
          return text;
        }
        case "elevator":
          return "Elevator";
        case "entrance": {
          let text = "Entrance/Door";
          if (s.automatic_door_type && s.automatic_door_type !== "no") {
            text += " (automatic door)";
          } else if (s.door_type && s.door_type !== "no") {
            text += " (door)";
          }
          return text;
        }
        case "cycle_barrier":
          return "Cycle barrier";
        default:
          return "-";
      }
    };

    const instructions = responseRoute.steps.map(function (s) {
      return {
        distance: s.distance,
        time: s.duration,
        text: getStepText(s),
        index: s.index,
      };
    });

    return {
      name: "",
      coordinates: coordinates,
      summary: {
        totalDistance: responseRoute.distance,
        totalTime: responseRoute.duration,
        totalAccessibility: responseRoute.accessibility,
        totalElevationUp: responseRoute.elevation_up,
        totalElevationDown: responseRoute.elevation_down,
      },
      waypoints: [coordinates[0], coordinates[coordinates.length - 1]],
      instructions: instructions,
    };
  },

  buildRouteRequest: function (waypoints, options, profile) {
    if (waypoints.length < 2) {
      return null;
    }

    const preview = !!options.geometryOnly;

    return {
      start: waypoints[0].latLng,
      destination: waypoints[waypoints.length - 1].latLng,
      profile: profile,
      include_infos: !preview,
      include_full_path: true,
      include_steps: true,
      include_steps_path: false,
      include_edges: false,
      include_statistics: true,
    };
  },
});

L.Routing.ppr = function (options) {
  return new L.Routing.PPR(options);
};
