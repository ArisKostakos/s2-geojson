package geo

import (
	"errors"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"github.com/golang/geo/s2"
	geojson "github.com/paulmach/go.geojson"
)

const (
	//EarthRadius the radius of earth in kilometers
	EarthRadius = 6371.01
	maxCells    = 100
)

// Point struct contains the lat/lng of a point
type Point struct {
	Lat float64
	Lng float64
}

// DecodeGeoJSON decodes a feature collection
func DecodeGeoJSON(json []byte) ([]*geojson.Feature, error) {
	f, err := geojson.UnmarshalFeatureCollection(json)
	if err != nil {
		return nil, err
	}
	return f.Features, nil
}

// PointsToPolygon converts points to s2 polygon
func PointsToPolygon(points [][]float64) *s2.Polygon {
	var pts []s2.Point
	for _, pt := range points {
		pts = append(pts, s2.PointFromLatLng(s2.LatLngFromDegrees(pt[1], pt[0])))
	}
	loop := s2.LoopFromPoints(pts)

	return s2.PolygonFromLoops([]*s2.Loop{loop})
}

// CoverPolygon converts s2 polygon to cell union and returns the respective cells
func CoverPolygon(p *s2.Polygon, maxLevel, minLevel int) (s2.CellUnion, []string, [][][]float64) {
	var tokens []string
	var s2cells [][][]float64

	rc := &s2.RegionCoverer{MaxLevel: maxLevel, MinLevel: minLevel, MaxCells: maxCells}
	r := s2.Region(p)
	covering := rc.Covering(r)

	for _, c := range covering {
		cell := s2.CellFromCellID(s2.CellIDFromToken(c.ToToken()))

		s2cells = append(s2cells, edgesOfCell(cell))

		tokens = append(tokens, c.ToToken())
	}
	return covering, tokens, s2cells
}

// CoverPoint converts a point to cell based on given level
func CoverPoint(p Point, maxLevel int) (s2.Cell, string, [][][]float64) {
	var s2cells [][][]float64

	cid := s2.CellFromLatLng(s2.LatLngFromDegrees(p.Lat, p.Lng)).ID().Parent(maxLevel)
	cell := s2.CellFromCellID(cid)
	token := cid.ToToken()

	s2cells = append(s2cells, edgesOfCell(cell))

	return cell, token, s2cells
}

func edgesOfCell(c s2.Cell) [][]float64 {
	var edges [][]float64
	for i := 0; i < 4; i++ {
		latLng := s2.LatLngFromPoint(c.Vertex(i))
		edges = append(edges, []float64{latLng.Lat.Degrees(), latLng.Lng.Degrees()})
	}
	return edges
}

// RawPoint is Polygon Points (lat,long).
type RawPoint = [][]float64

// DistanceFunc accepts two points and returns a distance as float64.
type DistanceFunc func(p1, p2 Point) float64

// CalculateEquirectangularDistance is a DistanceFunc calculating distance with the equirectangular formula.
func CalculateEquirectangularDistance(p1, p2 Point) float64 {
	x := toRadians(p2.Lng-p1.Lng) * math.Cos(toRadians(p1.Lat+p2.Lat)/2)
	y := toRadians(p2.Lat - p1.Lat)
	return math.Sqrt(x*x+y*y) * EarthRadius
}

func toRadians(v float64) float64 {
	return v * (math.Pi / 180.0)
}

// GeoJSONPointsToPolygon converts geo json points (lng,lat) to s2 Polygon.
func GeoJSONPointsToPolygon(points []RawPoint) []*s2.Polygon {
	s2Polygons := []*s2.Polygon{}
	for _, p := range points {
		s2p := parseGeoJSONPointsToS2Polygon(p)
		s2Polygons = append(s2Polygons, s2p)
	}
	return s2Polygons
}

func parseGeoJSONPointsToS2Polygon(points RawPoint) *s2.Polygon {
	var pts []s2.Point
	for _, pt := range points {
		pts = append(pts, s2.PointFromLatLng(s2.LatLngFromDegrees(pt[1], pt[0])))
	}
	loop := s2.LoopFromPoints(pts)

	return s2.PolygonFromLoops([]*s2.Loop{loop})
}

//PolygonContainsPoints checks if one or more points are inside the given polygon
func PolygonContainsPoints(polygon s2.Polygon, points ...Point) bool {
	for _, point := range points {
		p := s2.PointFromLatLng(s2.LatLngFromDegrees(point.Lat, point.Lng))
		if !polygon.ContainsPoint(p) {
			return false
		}
	}
	return true
}

//UnmarshalGeoJSON decodes the data into an array of floats
func UnmarshalGeoJSON(json []byte) ([]RawPoint, error) {
	fc, err := geojson.UnmarshalFeatureCollection(json)
	if err != nil {
		return nil, err
	}

	return ParseFeatureCollection(fc)
}

//ParseFeatureCollection decodes a feature collection into an array of floats
func ParseFeatureCollection(fc *geojson.FeatureCollection) ([]RawPoint, error) {
	ftrs := []RawPoint{}
	for _, f := range fc.Features {
		ftr, err := parseFeature(f)
		if err != nil {
			return nil, err
		}
		ftrs = append(ftrs, ftr)
	}

	return ftrs, nil
}

func parseFeature(f *geojson.Feature) (RawPoint, error) {
	if f == nil {
		return nil, errors.New("feature cannot be null")
	}
	if !f.Geometry.IsPolygon() {
		return nil, errors.New("only polygon type supported")
	}

	if len(f.Geometry.Polygon) == 0 {
		return nil, errors.New("cannot find the polygon")
	}

	return f.Geometry.Polygon[0], nil
}

// IsPointInsidePolygons checks if any of the given polygons contain a given point.
func IsPointInsidePolygons(loc Point, partnerPolygons []*s2.Polygon) bool {
	for _, p := range partnerPolygons {
		if PolygonContainsPoints(*p, loc) {
			return true
		}
	}

	return false
}

func pToString(p s2.Point) string {
	return fmt.Sprintf("\t(%0.5f\t%0.5f)", p.X, p.Y)
}

// GrowPolygon returns a new polygon, scaled with given buffer
func GrowPolygon(poly *s2.Polygon, buffer float64) (*s2.Polygon, error) {
	fmt.Println("GOT POLYGON:")
	loop := poly.Loops()[0]
	fmt.Println("points:")
	for i, v := range loop.Vertices() {
		fmt.Printf("point #%d:%s\n", i, pToString(v))
	}
	fmt.Println("edges:")
	edgeNormals := make([]r3.Vector, loop.NumVertices())
	for i := 0; i < loop.NumVertices(); i++ {
		var edge s2.Edge
		if i == loop.NumVertices()-1 {
			edge = s2.Edge{loop.Vertices()[i], loop.Vertices()[0]}
		} else {
			edge = s2.Edge{loop.Vertices()[i], loop.Vertices()[i+1]}
		}
		fmt.Printf("edge #%d: %s->%s\n", i, pToString(edge.V0), pToString(edge.V1))

		midPoint := s2.Point{edgeMidpoint(edge)}
		n1, _ := edgeNormal(edge)
		n1Norm := n1.Normalize()
		edgeNormals[i] = n1Norm
		_ = midPoint
		//fmt.Printf("center: %s\n", pToString(midPoint))
		fmt.Printf("normals: %s\n", pToString(s2.Point{n1Norm}))
		//fmt.Printf("center*normal: %s\n", pToString(s2.Point{midPoint.Add(n1Norm.Mul(buffer))}))
	}

	for i := 0; i < loop.NumVertices(); i++ {
		var edgeIdA, edgeIdB int

		if i == 0 {
			edgeIdA = loop.NumVertices() - 1
		} else {
			edgeIdA = i - 1
		}

		edgeIdB = i

		vec := loop.Vertices()[i]
		na := edgeNormals[edgeIdA]
		nb := edgeNormals[edgeIdB]

		bis := na.Add(nb).Normalize()

		l := buffer / math.Sqrt(1+na.Dot(nb))

		offsetVec := r3.Vector{vec.X + l*bis.X, vec.Y + l*bis.Y, 0}
		//bis = bis
		//fmt.Printf("Vertex #%d bis: %s\n", i, pToString(s2.Point{bis}))
		fmt.Printf("Vertex #%d new: %s\n", i, pToString(s2.Point{offsetVec}))
	}

	fmt.Println("\nMADE NEW GROWN POLYGON:")
	grownPoly := &s2.Polygon{}

	return grownPoly, nil
}

func edgeNormal(e s2.Edge) (r3.Vector, r3.Vector) {
	dx := e.V1.X - e.V0.X
	dy := e.V1.Y - e.V0.Y
	return r3.Vector{-dy, dx, 0}, r3.Vector{dy, -dx, 0}
}

func edgeMidpoint(e s2.Edge) r3.Vector {
	return r3.Vector{(e.V0.X + e.V1.X) / 2, (e.V0.Y + e.V1.Y) / 2, 0}
}

func PolygonToFeatureCollection(polygon *s2.Polygon) *geojson.FeatureCollection {
	var newPolygon [][][]float64
	z := polygon.Loops()[0]

	for _, point := range z.Vertices() {
		mv := r3.Vector{
			X: point.X,
			Y: point.Y,
			Z: point.Z,
		}
		mp := s2.LatLngFromPoint(s2.Point{mv})
		ep := [][]float64{
			{
				mp.Lng.Degrees(),
				mp.Lat.Degrees(),
			},
		}
		newPolygon = append(newPolygon, ep)
	}

	gm := geojson.NewPolygonFeature(newPolygon)
	ft := geojson.NewFeatureCollection()
	ft.AddFeature(gm)

	return ft
}