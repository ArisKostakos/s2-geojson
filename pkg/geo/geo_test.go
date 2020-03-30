package geo

import (
	"fmt"
	"testing"

	"github.com/golang/geo/r3"
	"github.com/golang/geo/s2"
	"github.com/stretchr/testify/assert"
)

var validJSON = []byte(`
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "properties": {},
      "geometry": {
        "type": "Polygon",
        "coordinates": [
          [
            [
              -71.98116712385131,
              40.58058466412764
            ],
            [
              -127.79249821831927,
              48.63290858589535
            ],
            [
              -90.17478214204795,
              25.64152637306577
            ],
            [
              -71.98116712385131,
              40.58058466412764
            ]
          ]
        ]
      }
    }
  ]
}
`)

func TestDecodeGeoJSON(t *testing.T) {
	r, err := DecodeGeoJSON(validJSON)
	assert.NoError(t, err)
	assert.True(t, r[0].Geometry.IsPolygon())

	_, err = DecodeGeoJSON([]byte("foo"))
	assert.Error(t, err)
}

func TestPointsToPolygon(t *testing.T) {
	r, err := DecodeGeoJSON(validJSON)
	assert.NoError(t, err)
	assert.True(t, r[0].Geometry.IsPolygon())

	p := PointsToPolygon(r[0].Geometry.Polygon[0])
	assert.Equal(t, 4, p.NumEdges())
}

func TestCoverPolygon(t *testing.T) {
	f, _ := DecodeGeoJSON(validJSON)
	p := PointsToPolygon(f[0].Geometry.Polygon[0])

	u, tk, c := CoverPolygon(p, 4, 1)
	assert.True(t, u.IsValid())
	assert.Equal(t, 22, len(tk))
	assert.Equal(t, 4, len(c[0]))

}

func TestCoverPoint(t *testing.T) {
	cell, token, edges := CoverPoint(Point{Lat: 38.34, Lng: 34.34}, 1)
	assert.Equal(t, "14", cell.ID().ToToken())
	assert.Equal(t, "14", token)
	assert.Equal(t, 4, len(edges[0]))

}

var l1 = []RawPoint{{
	{23.823165893554688, 38.036734877267705},
	{23.661460876464844, 38.031867399480674},
	{23.5821533203125, 37.915763487770754},
	{23.809089660644528, 37.90113599940821},
	{23.823165893554688, 38.036734877267705},
},
}

/*
var squarePolyPoints = []s2.Point{
	s2.Point{r3.Vector{-50, 50, 0}},
	s2.Point{r3.Vector{50, 50, 0}},
	s2.Point{r3.Vector{50, -50, 0}},
	s2.Point{r3.Vector{-50, -50, 0}},
}
*/

var squarePolyPoints = []s2.Point{
	s2.Point{r3.Vector{-50, 50, 0}},
	s2.Point{r3.Vector{-20, 70, 0}},
	s2.Point{r3.Vector{60, 10, 0}},
	s2.Point{r3.Vector{0, 0, 0}},
	s2.Point{r3.Vector{30, -60, 0}},
	s2.Point{r3.Vector{-100, -5, 0}},
}

func TestPolygonToFeatureCollection(t *testing.T) {
	loop := s2.LoopFromPoints(squarePolyPoints)
	polygon := s2.PolygonFromLoops([]*s2.Loop{loop})
	var s2Polygons []*s2.Polygon
	s2Polygons = append(s2Polygons, polygon)
	lala := PolygonToFeatureCollection(s2Polygons)
	yoyo, err := lala.MarshalJSON()
	if err != nil {
		fmt.Printf("err: %v\n", err)
	}
	fmt.Printf("Fc: %s\n", yoyo)
	t.Error()
}

func TestGrowPolygon(t *testing.T) {
	loop := s2.LoopFromPoints(squarePolyPoints)
	polygon := s2.PolygonFromLoops([]*s2.Loop{loop})
	GrowPolygon(polygon, 10)
	t.Error()
}
func TestGeoJsonPointsToPolygon(t *testing.T) {
	p := GeoJSONPointsToPolygon(l1)
	assert.False(t, p[0].IsEmpty())
}

func TestPolygonContainsPoints(t *testing.T) {
	p := GeoJSONPointsToPolygon(l1)
	p1 := Point{Lat: 38.00346733273856, Lng: 23.79329681396484}
	p2 := Point{Lat: 37.98317483351337, Lng: 23.755874633789062}
	actual := PolygonContainsPoints(*p[0], p1, p2)
	assert.True(t, actual)

	p3 := Point{Lat: 38.002385207833235, Lng: 23.854751586914062}
	actual = PolygonContainsPoints(*p[0], p1, p3)
	assert.False(t, actual)
}

func TestUnmarshalSinglePolygonGeoJSON(t *testing.T) {
	p, err := UnmarshalGeoJSON(getSinglePolyGeoJSON())
	assert.NoError(t, err)
	assert.NotEmpty(t, p)
	assert.Len(t, p, 1)
	assert.Len(t, p[0], 5)
	assert.Equal(t, l1, p)

	p, err = UnmarshalGeoJSON([]byte("foo"))
	assert.Error(t, err)

	p, err = UnmarshalGeoJSON(getPointGeoJSON())
	assert.Error(t, err)
}

func TestUnmarshalMultiPolygonGeoJSON(t *testing.T) {
	var expPoly = []RawPoint{{
		{-70.74440002441406, -33.442328356136564},
		{-70.74920654296875, -33.47125826950989},
		{-70.70148468017578, -33.47211742624329},
		{-70.70663452148438, -33.43258740206331},
		{-70.74440002441406, -33.442328356136564},
	}, {
		{-70.67367553710938, -33.42399153452155},
		{-70.68260192871094, -33.457510603405886},
		{-70.65067291259766, -33.42112605618061},
		{-70.67367553710938, -33.42399153452155},
	},
	}
	m, err := UnmarshalGeoJSON(getMultiPolyGeoJSON())
	assert.NoError(t, err)
	assert.Len(t, m, 2)
	assert.Len(t, m[0], 5)
	assert.Len(t, m[1], 4)
	assert.Equal(t, expPoly, m)

	m, err = UnmarshalGeoJSON(getSecondPointGeoJSON())
	assert.Error(t, err)
}

func getSinglePolyGeoJSON() []byte {
	return []byte(`
  {
"type": "FeatureCollection",
"features": [
  {
    "type": "Feature",
    "properties": {},
    "geometry": {
      "type": "Polygon",
      "coordinates": [
        [
          [
            23.823165893554688,
            38.036734877267705
          ],
          [
            23.661460876464844,
            38.031867399480674
          ],
          [
            23.5821533203125,
            37.915763487770754
          ],
          [
            23.809089660644528,
            37.90113599940821
          ],
          [
            23.823165893554688,
            38.036734877267705
          ]
        ]
      ]
    }
  }
]
}`)
}

func getMultiPolyGeoJSON() []byte {
	return []byte(`
  {
    "type": "FeatureCollection",
    "features": [
      {
        "type": "Feature",
        "properties": {},
        "geometry": {
          "type": "Polygon",
          "coordinates": [
            [
              [
                -70.74440002441406,
                -33.442328356136564
              ],
              [
                -70.74920654296875,
                -33.47125826950989
              ],
              [
                -70.70148468017578,
                -33.47211742624329
              ],
              [
                -70.70663452148438,
                -33.43258740206331
              ],
              [
                -70.74440002441406,
                -33.442328356136564
              ]
            ]
          ]
        }
      },
      {
        "type": "Feature",
        "properties": {},
        "geometry": {
          "type": "Polygon",
          "coordinates": [
            [
              [
                -70.67367553710938,
                -33.42399153452155
              ],
              [
                -70.68260192871094,
                -33.457510603405886
              ],
              [
                -70.65067291259766,
                -33.42112605618061
              ],
              [
                -70.67367553710938,
                -33.42399153452155
              ]
            ]
          ]
        }
      }
    ]
  }
  `)
}

func getPointGeoJSON() []byte {
	return []byte(`
  {
"type": "FeatureCollection",
"features": [
  {
    "type": "Feature",
    "properties": {},
    "geometry": {
      "type": "Point",
      "coordinates": [
        23.784713745117188,
        38.0091482264894
      ]
    }
  }
]
}`)
}

func getSecondPointGeoJSON() []byte {
	return []byte(`
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "properties": {},
      "geometry": {
        "type": "Polygon",
        "coordinates": [
          [
            [
              -70.74440002441406,
              -33.442328356136564
            ],
            [
              -70.74920654296875,
              -33.47125826950989
            ],
            [
              -70.70148468017578,
              -33.47211742624329
            ],
            [
              -70.70663452148438,
              -33.43258740206331
            ],
            [
              -70.74440002441406,
              -33.442328356136564
            ]
          ]
        ]
      }
    },
    {
      "type": "Feature",
      "properties": {},
      "geometry": {
        "type": "Point",
        "coordinates": [
          [
            [
              -70.67367553710938,
              -33.42399153452155
            ],
            [
              -70.68260192871094,
              -33.457510603405886
            ],
            [
              -70.65067291259766,
              -33.42112605618061
            ],
            [
              -70.67367553710938,
              -33.42399153452155
            ]
          ]
        ]
      }
    }
  ]
}
`)
}

func getNoFeaturesGeoJSON() []byte {
	return []byte(`
  {
"type": "FeatureCollection",
"invalid": [
  {
    "type": "Polygon",
    "properties": {},
    "geometry": {
      "type": "Point",
      "coordinates": [
        23.784713745117188,
        38.0091482264894
      ]
    }
  }
]
}`)
}
