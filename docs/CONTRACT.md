# Marps engine contract (v0.1)

Interface between the navigation engine and any frontend (game engine, web viewer, hardware HUD). The same JSON shapes work whether the engine is used as a library or over HTTP.

## Conventions

- **Coordinates**: local meters, Unity-style convention. `x` = east, `z` = north, origin at the southwest corner of the map (2000 m × 2000 m).
  - Conversion from the source projection: `x = X + 900`, `z = Y - 2567800`.
  - Unity: `x` and `z` as-is; `y` = elevation + 3623.578 (the response gives `elevationM` relative to the areoid, without that offset).
- **Bearings**: degrees. `0` = north (+z), `90` = east (+x), clockwise.
- **Times**: minutes for walking time; seconds for `missionTimeS` (only echoed back, the engine does not use it yet).
- **Nulls**: `null` fields are omitted from the JSON. E.g. `nextWaypoint` does not appear when the mission is complete.
- **Names**: camelCase.
- **Enumerations**: place types are `base`, `science`, `antenna`, `hazard`. Alert severities are `info`, `warning`, `danger`. Hazard severities are `high` (slope over 22°) and `moderate`.

## Input: `NavigationRequest` (frontend → engine, every frame or at 5–10 Hz)

```json
{
  "sessionId": "astronaut-1",
  "x": 300,
  "z": 1700,
  "headingDeg": 90,
  "pitchDeg": -5,
  "missionTimeS": 3600,
  "activeWaypointId": null,
  "forceReplan": false
}
```

| field | type | required | notes |
|---|---|---|---|
| `sessionId` | string | no (default `"default"`) | one session per astronaut: stores completed objectives and the cached route |
| `x`, `z` | number | yes | local position in meters |
| `headingDeg` | number | yes | look direction |
| `pitchDeg` | number | no | **reserved**: not used by the engine yet |
| `missionTimeS` | number | no | echoed back as-is |
| `activeWaypointId` | string | no | if sent, jumps to that step (e.g. `"B"` to return to base) |
| `forceReplan` | bool | no | forces a route recalculation |
| `stopOrder` | string[] | no | new visiting order for the stops still ahead (see below) |
| `emergencyReturn` | bool | no | emergency button: guide the astronaut straight to the base (see below) |
| `cancelEmergency` | bool | no | leave emergency mode and resume the mission |

### Reordering stops (`stopOrder`)

- The listed stops are visited first, in the given order; stops not listed keep their relative order after them. `["P3"]` moves P3 to the front; `["P4", "P2", "P3", "P1"]` is a full reorder.
- Completed stops and the departure / return base are fixed and ignored if listed, so the frontend can resend the same list safely.
- The route to the new active stop is recalculated on the same call (`route.replanned = true`). `objectives` comes back in the new order.
- Unknown ids, places that are not mission stops, empty ids or duplicates reject the whole list: the order is left unchanged and an `INVALID_STOP_ORDER` alert is returned.
- The order is per session.

### Emergency return (`emergencyReturn` / `cancelEmergency`)

- Send `emergencyReturn: true` once (button press). The mode stays on until `cancelEmergency: true`; it is per session.
- The target becomes the base (the departure step if it is a base, else the return step, else any place of type `base`). The route is computed on that same call even with background replanning.
- While active: stops passed on the way are not credited, `activeWaypointId` is ignored, pending stops stay `pending` and the return step is `active`. `stopOrder` still applies, for when the mission resumes.
- The response carries `emergency: { active, baseId, arrived, activatedAtS }` (omitted when not active) and an `EMERGENCY_RETURN` alert first in the list (`danger`, or `info` once `arrived`).
- On cancel, navigation resumes at the stop that was active.
- A mission without a base returns a `NO_BASE` alert and stays in normal mode.

Optional space weather fields (`spaceWeather`: `{ "stormActive": true, "level": "G3", "source": "DONKI" }`) are filled in by a weather provider, not by the frontend.

## Output: `NavigationResponse` (engine → frontend)

Example (position at base, looking east):

```json
{
  "sessionId": "example",
  "missionTimeS": 3600,
  "missionComplete": false,
  "position": { "x": 300, "z": 1700, "elevationM": -3612.99, "inside": true },
  "route": {
    "target": "P1",
    "found": true,
    "replanned": true,
    "lengthM": 478.34,
    "remainingM": 479.45,
    "estimatedMinutes": 8.96,
    "ascentM": 23.33,
    "descentM": 4.17,
    "maxSlopeDeg": 16.09,
    "points": [ { "x": 300, "z": 1700 }, { "x": 300.78, "z": 1699.22 }, "…" ]
  },
  "nextWaypoint": {
    "id": "P1", "type": "science", "name": "Glacier edge ridges",
    "x": 750, "z": 1700,
    "distanceM": 450, "bearingDeg": 90, "relativeBearingDeg": 0
  },
  "objectives": [
    { "step": 0, "id": "B",  "name": "Base (departure and return)", "status": "done",    "x": 300,  "z": 1700 },
    { "step": 1, "id": "P1", "name": "Glacier edge ridges", "status": "active", "x": 750, "z": 1700 },
    { "step": 2, "id": "P2", "status": "pending", "x": 1450, "z": 1400 },
    { "step": 3, "id": "P3", "status": "pending", "x": 780,  "z": 800 },
    { "step": 4, "id": "P4", "status": "pending", "x": 130,  "z": 230 },
    { "step": 5, "id": "B",  "status": "pending", "x": 300,  "z": 1700 }
  ],
  "hazards": [],
  "alerts": [ { "code": "SIGNAL_LIMITED", "severity": "info", "message": "Only 1 mast with line of sight." } ],
  "layers": {
    "slopeDeg": 5.81, "hazardClass": 0, "elevationM": -3612.99, "visibleAntennas": 1,
    "antennas": [ { "id": "B", "visible": true }, { "id": "A2", "visible": false }, { "id": "A3", "visible": false } ]
  },
  "computeMs": 35.9
}
```

### What the HUD uses

- **Next target**: `nextWaypoint.distanceM` and `relativeBearingDeg` (negative = left, positive = right). For the HUD arrow, the sign and magnitude are enough.
- **Route**: `route.points` is the remaining leg, from the current position, simplified to 0.5 m. Draw it as a polyline.
- **Alerts** (`alerts[].code`):
  - `HAZARD_AHEAD` (danger/warning): terrain >22° or rough within the next 40 m in the look direction.
  - `SIGNAL_LOSS` (warning): no mast with line of sight.
  - `SIGNAL_LIMITED` (info): only one.
  - `SOLAR_STORM` (warning): only if the weather provider reports it.
  - `NO_ROUTE` (danger): no safe route to the target.
  - `OFF_MAP` (warning): outside the map.
  - `EMERGENCY_RETURN` (danger, or info on arrival): emergency return to base is active.
  - `NO_BASE` (danger): emergency requested but the mission has no base.
  - `INVALID_STOP_ORDER` (warning): the `stopOrder` sent was rejected.
- **Nearby hazards** (`hazards`): zones within 150 m, sorted by distance. `severity` = `high` if any slope exceeds 22°.
- **Objectives** (`objectives[].status`): `done`, `active`, `pending`. The base appears on departure (step 0) and on return (last step).
- **Layers** (`layers`): slope underfoot, hazard class, elevation, masts in line of sight.

## Engine interface: `INavigationEngine`

Frontends depend on `Marps.Engine.Abstractions.INavigationEngine` (implemented by `NavigationEngine`, thread-safe), not on the concrete class. `NavigationService` also takes the interface.

| member | use |
|---|---|
| `Map`, `Mission`, `Base` | terrain, mission and the emergency base (null if none) |
| `Update(NavigationRequest)` | one navigation step (all request fields above) |
| `ReorderStops(current, stopOrder)` | same as sending `stopOrder`; returns the updated step |
| `StartEmergencyReturn(current)` / `CancelEmergencyReturn(current)` | same as `emergencyReturn` / `cancelEmergency` |
| `PlanRoute(from, to)` | safe route between two points, no session |
| `PlanItinerary(from, placeIds)` | every leg through mission places, in order |
| `PlanTrip(TripRequest)` / `PlanTrip(current, stops, destination)` | route from the current point through the stops to the destination (below) |

The convenience methods copy the request they are given; the caller's object is never changed.

Points that are NaN or infinite never produce a route: `PlanRoute`, every itinerary leg and `Update` report "not found" (`NO_ROUTE`) instead of routing from an arbitrary node. Finite points off the map are clamped to the nearest border node (the response flags `OFF_MAP`). After `Dispose`, the engine keeps working and plans on the caller's thread.

### Trip planning (`PlanTrip`)

Input (`TripRequest`):

```json
{
  "x": 300, "z": 1700,
  "stops": [ { "id": "P2" }, { "x": 1000, "z": 1000 } ],
  "destination": { "id": "B" }
}
```

- A stop or destination is a mission place (`id`) or a free point (`x`, `z`). If both are sent, `id` wins. `stops` may be empty (direct route).
- Output (`TripPlan`): `found` (every leg has a safe route), totals (`lengthM`, `estimatedMinutes`, `ascentM`, `descentM`, `maxSlopeDeg`), `legs[]` (`index`, `toId` — null for free points —, `fromX/Z`, `toX/Z`, `found`, statistics and `points`), the joined `points` from the current point to the destination, and `computeMs`.
- A leg with no safe route has `found: false` and no points. The next leg still starts at that stop, so the rest of the trip is reported. The joined `points` is empty when any leg is missing.
- Bad input (no destination, unknown id, a stop with neither id nor finite `x`/`z`, a stop or destination outside the map, non-finite position) does not throw: `error` explains it and no search runs. The current position may be slightly off the map, as in navigation.
- It does not touch any session: it is a plan, not navigation.

## Harness HTTP API (testing only)

| method | path | use |
|---|---|---|
| GET | `/api/health` | status |
| GET | `/api/mission` | map (hazard and slope in base64), places, zones, reference legs |
| POST | `/api/route` | `{fromX, fromZ, toX, toZ}` → route and statistics |
| POST | `/api/sensor` | sets the test astronaut's state |
| GET | `/api/hud` | runs one service step and returns `{ hud, serverMs }` |
| POST | `/api/navigate` | shortcut: `NavigationRequest` → `{ hud, serverMs }` |
| POST | `/api/trip` | `TripRequest` → `{ trip: TripPlan, serverMs }`; bad input → 400 `{ error }` |
| POST | `/api/itinerary` | `{fromX, fromZ, order: [ids]}` → every leg through the given places, with totals (reorder preview) |

## Data files

| file | content |
|---|---|
| `mission.json` | mission name and objective, terrain bounds, step order, places (`id`, `type`, `name`, `X`, `Y`, `reason`) |
| `hazards.json` | hazard zones (`type`, `area_m2`, `center`, `box_ULX_ULY_LRX_LRY`, `max_slope_deg`, optional `height_m`, `fraction_over_15`) |
| `reference_routes.json` | reference legs used to validate the routing model (`with_rule`, `without_rule`) |

## Pending / assumptions to validate

- `AntennaHeightM = 3 m` and `EyeHeightM = 1.7 m`: **assumptions**, not in the data.
- Time model: fitted to 5 reference legs (max error 0.11 min). Validate with field measurements.
- Roughness (hazard 3): no extra penalty (`RoughFactor = 1`). Still to be decided.
- Space weather and SWIM/SHARAD/THEMIS layers: the engine has the `ISpaceWeatherProvider` interface; providers are not implemented yet.
- `pitchDeg` is not used yet.
