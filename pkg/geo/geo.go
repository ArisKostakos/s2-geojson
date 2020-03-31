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

// PointsToPolygon2 converts points to s2 polygon
func PointsToPolygon2(points [][]float64) *s2.Polygon {
	var pts []s2.Point
	for _, pt := range points {
		pts = append(pts, s2.Point{r3.Vector{pt[0], pt[1], 0}})
	}
	loop := s2.LoopFromPoints(pts)

	return s2.PolygonFromLoops([]*s2.Loop{loop})
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

func pToString2(p [][]float64) string {
	return fmt.Sprintf("\t(%0.5f\t%0.5f)", p[0][0], p[0][1])
}

// GrowPolygon2 returns a new polygon, scaled with given buffer
func GrowPolygon2(poly [][][]float64, buffer float64) ([][][]float64, error) {
	fmt.Println("GOT POLYGON:")
	fmt.Println("points:")

	loopVertices := make([]s2.Point, 0, 1000)

	//Convert To Points
	for i, v := range poly {
		for j, p := range v {
			//fmt.Printf("point #%d:%s\n", i, pToString2(v))
			fmt.Printf("i:%d, j:%d, p0:%f, p1:%f\n", i, j, p[0], p[1])

			loopVertices = append(loopVertices, s2.Point{r3.Vector{p[0], p[1], 0}})

		}
	}

	fmt.Println("edges:")
	edgeNormals := make([]r3.Vector, len(loopVertices))
	for i := 0; i < len(loopVertices); i++ {
		var edge s2.Edge
		if i == len(loopVertices)-1 {
			edge = s2.Edge{loopVertices[i], loopVertices[0]}
		} else {
			edge = s2.Edge{loopVertices[i], loopVertices[i+1]}
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

	grownPolyPoints := make([][]float64, 0, len(loopVertices))
	for i := 0; i < len(loopVertices); i++ {
		var edgeIdA, edgeIdB int

		if i == 0 {
			edgeIdA = len(loopVertices) - 1
		} else {
			edgeIdA = i - 1
		}

		edgeIdB = i

		vec := loopVertices[i]
		na := edgeNormals[edgeIdA]
		nb := edgeNormals[edgeIdB]

		bis := na.Add(nb).Normalize()

		l := buffer / math.Sqrt(1+na.Dot(nb))

		offsetVec := r3.Vector{vec.X + l*bis.X, vec.Y + l*bis.Y, vec.Z}
		//bis = bis
		//fmt.Printf("Vertex #%d bis: %s\n", i, pToString(s2.Point{bis}))
		fmt.Printf("Vertex #%d new: %s\n", i, pToString(s2.Point{offsetVec}))
		grownPolyPoints = append(grownPolyPoints, []float64{offsetVec.X, offsetVec.Y})
	}

	fmt.Println("\nMADE NEW GROWN POLYGON:")

	//grownLoop := s2.LoopFromPoints(grownPolyPoints)
	//grownPolygon := s2.PolygonFromLoops([]*s2.Loop{grownLoop})

	return [][][]float64{grownPolyPoints}, nil
}

func pToString(p s2.Point) string {
	return fmt.Sprintf("\t(%0.5f\t%0.5f)", p.X, p.Y)
}

// GrowPolygon returns a new polygon, scaled with given buffer
func GrowPolygon(poly *s2.Polygon, bufferKm float64) (*s2.Polygon, error) {
	fmt.Println("GOT POLYGON:")
	loop := poly.Loops()[0]

	//IF FIRST AND LASTR THE SAME, IGNORE LAST
	loop = s2.LoopFromPoints(loop.Vertices()[:loop.NumVertices()-1])

	fmt.Println("points:")
	for i, v := range loop.Vertices() {
		fmt.Printf("point #%d:%s\n", i, pToString(v))
	}
	fmt.Println("edges:")
	edgeNormals := make([]r3.Vector, loop.NumVertices())
	oldEdges := make([]s2.Edge, loop.NumVertices())
	for i := 0; i < loop.NumVertices(); i++ {
		var edge s2.Edge
		if i == loop.NumVertices()-1 {
			edge = s2.Edge{loop.Vertices()[i], loop.Vertices()[0]}
		} else {
			edge = s2.Edge{loop.Vertices()[i], loop.Vertices()[i+1]}
		}
		fmt.Printf("edge #%d: %s->%s\n", i, pToString(edge.V0), pToString(edge.V1))
		oldEdges[i] = edge
		midPoint := s2.Point{edgeMidpoint(edge)}
		_, n1 := edgeNormal(edge)
		n1Norm := n1.Normalize()
		edgeNormals[i] = n1Norm
		_ = midPoint
		//fmt.Printf("center: %s\n", pToString(midPoint))
		fmt.Printf("normals: %s\n", pToString(s2.Point{n1Norm}))
		//fmt.Printf("center*normal: %s\n", pToString(s2.Point{midPoint.Add(n1Norm.Mul(buffer))}))
	}

	grownPolyPoints := make([]s2.Point, 0, loop.NumVertices()*3)
	for i, e := range oldEdges {
		a := s2.Point{r3.Vector{e.V0.X + kmToLng(bufferKm, e.V0.Y)*edgeNormals[i].X, e.V0.Y + kmToLat(bufferKm)*edgeNormals[i].Y, 0}}
		b := s2.Point{r3.Vector{e.V1.X + kmToLng(bufferKm, e.V1.Y)*edgeNormals[i].X, e.V1.Y + kmToLat(bufferKm)*edgeNormals[i].Y, 0}}

		//Create corner vertex
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

		cNormal := na.Add(nb).Normalize()

		//l := buffer / math.Sqrt(1+na.Dot(nb))

		offsetVec := r3.Vector{vec.X + kmToLng(bufferKm, vec.Y)*cNormal.X, vec.Y + kmToLat(bufferKm)*cNormal.Y, 0}
		//bis = bis
		//fmt.Printf("Vertex #%d bis: %s\n", i, pToString(s2.Point{bis}))
		//fmt.Printf("Vertex #%d new: %s\n", i, pToString(s2.Point{offsetVec}))
		grownPolyPoints = append(grownPolyPoints, s2.Point{offsetVec})

		grownPolyPoints = append(grownPolyPoints, a)
		grownPolyPoints = append(grownPolyPoints, b)
	}
	/*
		newEdges := make([]s2.Edge, 0, loop.NumVertices())
		newEdges = append(newEdges, s2.Edge{a, b})
		for i, newvec := range grownPolyPoints {

		}*/

	//fmt.Println("Num of edges:", len(newEdges))
	//Eliminate Crossings
	//grownPolyPoints2 := make([]s2.Point, 0, loop.NumVertices()*2)
	/*
		for i := 0; i < len(newEdges)-1; i++ {
			ii := i
			for j := i + 2; j < len(newEdges); j++ {
				jj := j
				edgeI := newEdges[ii]
				edjeJ := newEdges[jj]
				crossing, crossPoint := checkEdgeCross(edgeI, edjeJ)
				if crossing {
					fmt.Printf("Cross between %d and %d at point %v\n", i, j, crossPoint)
					fmt.Printf("Edge #%d: %v, Edge #%d: %v\n", i, edgeI, j, edjeJ)
					break
				}
			}
		}*/

	/*
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

			offsetVec := r3.Vector{vec.X + l*bis.X, vec.Y + l*bis.Y, vec.Z}
			//bis = bis
			//fmt.Printf("Vertex #%d bis: %s\n", i, pToString(s2.Point{bis}))
			fmt.Printf("Vertex #%d new: %s\n", i, pToString(s2.Point{offsetVec}))
			grownPolyPoints = append(grownPolyPoints, s2.Point{offsetVec})
		}*/

	fmt.Println("\nMADE NEW GROWN POLYGON:")

	grownLoop := s2.LoopFromPoints(grownPolyPoints)
	grownPolygon := s2.PolygonFromLoops([]*s2.Loop{grownLoop})

	return grownPolygon, nil
}

func checkEdgeCross(e1, e2 s2.Edge) (bool, s2.Point) {
	/*
		if s2.CrossingSign(e1.V0, e1.V1, e2.V0, e2.V1) == s2.Cross {
			fmt.Println("CROSSING")
			return true, s2.Intersection(e1.V0, e1.V1, e2.V0, e2.V1)
		}

		return false, s2.Point{}
	*/
	var val float64 = 1000
	e1p0 := s2.Point{r3.Vector{e1.V0.X * val, e1.V0.Y * val, 0}}
	e1p1 := s2.Point{r3.Vector{e1.V1.X * val, e1.V1.Y * val, 0}}
	e2p0 := s2.Point{r3.Vector{e2.V0.X * val, e2.V0.Y * val, 0}}
	e2p1 := s2.Point{r3.Vector{e2.V1.X * val, e2.V1.Y * val, 0}}
	l1 := CreateLine(e1p0, e1p1)
	l2 := CreateLine(e2p0, e2p1)

	result, err := Intersection(l1, l2)
	if err != nil {
		fmt.Println("NOOOOOOOOO")
		return false, s2.Point{}

	}
	fmt.Println("YESSSSSS")
	return true, result

	/*
		var val float64 = 1000
		e1p0 := s2.Point{r3.Vector{e1.V0.X * val, e1.V0.Y * val, 0}}
		e1p1 := s2.Point{r3.Vector{e1.V1.X * val, e1.V1.Y * val, 0}}
		e2p0 := s2.Point{r3.Vector{e2.V0.X * val, e2.V0.Y * val, 0}}
		e2p1 := s2.Point{r3.Vector{e2.V1.X * val, e2.V1.Y * val, 0}}
		if s2.CrossingSign(e1p0, e1p1, e2p0, e2p1) == s2.Cross {
			fmt.Println("CROSSING")
			return true, s2.Intersection(e1p0, e1p1, e2p0, e2p1)
		}

		return false, s2.Point{}*/

}

func kmToLat(km float64) float64 {
	return (km / EarthRadius) * (180 / math.Pi)
}

func kmToLng(km float64, theta float64) float64 {
	return (km / EarthRadius) * (180 / math.Pi) / math.Cos(theta*math.Pi/180)
}

type Line struct {
	slope float64
	yint  float64
}

func CreateLine(a, b s2.Point) Line {
	slope := (b.Y - a.Y) / (b.X - a.X)
	yint := a.Y - slope*a.X
	return Line{slope, yint}
}

func EvalX(l Line, x float64) float64 {
	return l.slope*x + l.yint
}

func Intersection(l1, l2 Line) (s2.Point, error) {
	if l1.slope == l2.slope {
		return s2.Point{}, errors.New("The lines do not intersect")
	}
	x := (l2.yint - l1.yint) / (l1.slope - l2.slope)
	y := EvalX(l1, x)
	return s2.Point{r3.Vector{x, y, 0}}, nil
}

func edgeNormal(e s2.Edge) (r3.Vector, r3.Vector) {
	dx := e.V1.X - e.V0.X
	dy := e.V1.Y - e.V0.Y
	return r3.Vector{-dy, dx, 0}, r3.Vector{dy, -dx, 0}
}

func edgeMidpoint(e s2.Edge) r3.Vector {
	return r3.Vector{(e.V0.X + e.V1.X) / 2, (e.V0.Y + e.V1.Y) / 2, 0}
}

func PolygonToFeatureCollection(polygon []*s2.Polygon) *geojson.FeatureCollection {
	var newPolygon [][][]float64
	for _, polygon := range polygon {
		for _, loop := range polygon.Loops() {

			var points [][]float64
			for _, vertex := range loop.Vertices() {

				mv := r3.Vector{
					X: vertex.X,
					Y: vertex.Y,
					Z: vertex.Z,
				}
				//mp := s2.LatLngFromPoint(s2.Point{mv})
				ep := []float64{
					mv.X,
					mv.Y,
				}
				points = append(points, ep)
			}

			newPolygon = append(newPolygon, points)
		}
	}

	gm := geojson.NewPolygonFeature(newPolygon)
	ft := geojson.NewFeatureCollection()
	ft.AddFeature(gm)

	return ft
}

func PolygonToFeatureCollectionBackup(polygon []*s2.Polygon) *geojson.FeatureCollection {
	var newPolygon [][][]float64
	for _, polygon := range polygon {
		for _, loop := range polygon.Loops() {

			var points [][]float64
			for _, vertex := range loop.Vertices() {

				mv := r3.Vector{
					X: vertex.X,
					Y: vertex.Y,
					Z: vertex.Z,
				}
				mp := s2.LatLngFromPoint(s2.Point{mv})
				ep := []float64{
					mp.Lat.Degrees(),
					mp.Lng.Degrees(),
				}
				points = append(points, ep)
			}

			newPolygon = append(newPolygon, points)
		}
	}

	gm := geojson.NewPolygonFeature(newPolygon)
	ft := geojson.NewFeatureCollection()
	ft.AddFeature(gm)

	return ft
}

// GrowPolygonBackup returns a new polygon, scaled with given buffer
func GrowPolygonBackup(poly *s2.Polygon, buffer float64) (*s2.Polygon, error) {
	fmt.Println("GOT POLYGON:")
	loop := poly.Loops()[0]

	//IF FIRST AND LASTR THE SAME, IGNORE LAST
	loop = s2.LoopFromPoints(loop.Vertices()[:loop.NumVertices()-1])

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
		_, n1 := edgeNormal(edge)
		n1Norm := n1.Normalize()
		edgeNormals[i] = n1Norm
		_ = midPoint
		//fmt.Printf("center: %s\n", pToString(midPoint))
		fmt.Printf("normals: %s\n", pToString(s2.Point{n1Norm}))
		//fmt.Printf("center*normal: %s\n", pToString(s2.Point{midPoint.Add(n1Norm.Mul(buffer))}))
	}

	grownPolyPoints := make([]s2.Point, 0, loop.NumVertices())
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

		offsetVec := r3.Vector{vec.X + l*bis.X, vec.Y + l*bis.Y, vec.Z + l*bis.Z}
		//bis = bis
		//fmt.Printf("Vertex #%d bis: %s\n", i, pToString(s2.Point{bis}))
		fmt.Printf("Vertex #%d new: %s\n", i, pToString(s2.Point{offsetVec}))
		grownPolyPoints = append(grownPolyPoints, s2.Point{offsetVec})
	}

	fmt.Println("\nMADE NEW GROWN POLYGON:")

	grownLoop := s2.LoopFromPoints(grownPolyPoints)
	grownPolygon := s2.PolygonFromLoops([]*s2.Loop{grownLoop})

	return grownPolygon, nil
}
