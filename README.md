# libsemantic

Semantic world manipulation library.

# File format

## On-disk representation

Semantic world on-disk format is JSON file with following structure. There may
be a binary representation added in the future.

## Definitions

This documentation uses pseudo format for documentation purposes:

 * `{}`: JSON object dictionary
 * `[]` JSON array
 * `String`: string data type
 * `Int`: integral JSON number integer
 * `Double`: real JSON number double
 * `Boolean`: boolean JSON value
 * `Array<type>`: unbounded array of given type(s)
 * `Array<type, size>`: bounded array of given type(s) and size
 * `Optional` given entry is optional
 * `Enum` string data type limited to enumerated values
 * `Any` any data type
 * `?` no fixed name

Special types:
 * `SRS` string with spatial reference system. Supports Proj.4 string, EPSG:code, EPSG:code+code, WKT string and custom [ENU](https://github.com/melowntech/true3d-format-spec/blob/master/enu.md).
 * `Point3` alias to Array<Double, 3>, point in Cartesian space [ x, y, z ]
 * `Size2` alias to Array<Double, 2>, [ width, height ]

SRS must be a cartesian system. Geodetic system makes no sense.

Unless explicitely stated the default spatial unit is a SRS unit. If SRS is a
projected system (e.g. UTM) all Z coordinates are vertically adjusted to account
for horizontal scale.

## Outline

```javascript
World = {
    SRS srs                   // Spatial reference
    Point3 origin             // A point all entities in the world are relative
    Array<Building> buildings // List of all buildings in the world
    Array<Tree> trees         // List of all trees in the world
}
```

## Entities

### Common entity attributes

```javascript
Entity = {
    String id                   // Entity ID
    Optional String descriptor  // World unique descritpor (TBD)
    Point3 origin               // All coordinates in the entity are relative to
                                // World.origin + origin
}

```

### Building

Building is currently defined only by its roofs. Facades are implicite. There
are two versatile roof types: rectangular and ciruclar. Nomenclature for
circular roof follow the nomenclature of rectangular roof, i.e. _ridge_ means
_pinnalce_ and _eave_ means _rim_.

```javascript
Rectangular = {
    String type = "rectangular"
    Point3 center             // Roof center point, relative to Building.origin
                              // Z coordinate marks base for implicit facade
    Size2 size                // Roof base rectangle footprint dimensions
    Array<Double, 2> skew     // [ top, bottom ] skew [rad]
    Double azimuth            // roof orientation angle [rad]
    Array<Double, 4> curb     // [ top, bottom, left, right ] curb distance from center (0-1)
    Array<Double, 2> hip      // [ top, bottom ] ridge hip distance from center (0-1)
    heights {                 // Heights (above center[2])
        Array<Double, 4> eave // [ top, bottom, left, right ] eave height
        Double curb           // Common height of all 4 curbs
        Double ridge          // Ridge height
    }
}
```

```javascript
Circular = {
    String type = "circular"
    Point3 center             // Roof center point, relative to Building.origin
                              // Z coordinate marks base for implicit facade
    Double radius             // Circlular footprint radius
    Double curb               // curb distance from center (0-1)
    heights {                 // Heights (above center[2])
       Double eave            // Eave (circle rim) height
       Double curb            // Curb height
       Double ridge           // Ridge (pinnacle) height
   }
}
```

```javascript
Building : Entity = {
    Array<Rectantular|Circular> roofs // List of all building's roofs. At least one.
}
```

### Tree

```javascript
Tree : Entity = {
    Point3 center             // Tree center relative to Entity.origin
    Double a                  // Semi-major axis of the base spheroid
    Double b                  // Semi-minor axis of the base spheroid
    Array<Double> harmonics   // Spherical harmonics parameters
}
```
