package controllers

import (
	"fmt"
	"strconv"

	"github.com/gin-gonic/gin"
	"github.com/golang/geo/s2"
	"github.com/pantrif/s2-geojson/pkg/geo"
)

// GeometryController struct
type GeometryController struct{}

// GetBuffed bluh...
func (u GeometryController) GetBuffed(c *gin.Context) {
	gJSON := []byte(c.PostForm("geojson"))
	bufferInMeters, err := strconv.Atoi(c.PostForm("buffer_meters"))
	//fmt.Printf("Received: %s\n", gJSON)
	//fmt.Printf("Received: %d\n", bufferInMeters)
	/*
		fffcccc, err := geojson.UnmarshalFeatureCollection(gJSON)
		if err != nil {
			log.Warnf("failed to unmarshal partner polygon with err: %v", err)
		}*/
	/*
		pppppp, err := geo.ParseFeatureCollection(fffcccc)
		if err != nil {
			log.Warnf("failed to parse partner polygon with err: %v", err)
		}
	*/
	//vallll := geo.GeoJSONPointsToPolygon(pppppp)

	fs, err := geo.DecodeGeoJSON(gJSON)

	if err != nil {
		c.JSON(400, gin.H{
			"error": err.Error(),
		})
		return
	}

	//var tokens []string
	//var s2cells [][][]float64

	grownPolys := []*s2.Polygon{}
	buffkm := (float64(bufferInMeters) / 1000)
	for _, f := range fs {

		if f.Geometry.IsPolygon() {
			for _, poly := range f.Geometry.Polygon {
				p := geo.PointsToPolygon2(poly)
				fmt.Printf("Poly: %v", p)
				grownPoly, err := geo.GrowPolygon(p, buffkm)
				if err != nil {
					fmt.Errorf("couldn't grow the poly: %v\n", err)
					continue
				}
				grownPolys = append(grownPolys, grownPoly)

				//_, t, c := geo.CoverPolygon(p, maxLevel, minLevel)
				//s2cells = append(s2cells, c...)
				//tokens = append(tokens, t...)
			}
		}
		if f.Geometry.IsPoint() {
			point := geo.Point{Lat: f.Geometry.Point[1], Lng: f.Geometry.Point[0]}
			_ = point
			//_, t, c := geo.CoverPoint(point, maxLevel)
			//s2cells = append(s2cells, c...)
			//tokens = append(tokens, t)
		}
	}

	/*
		grownPolys := []*s2.Polygon{}
		for _, normalPoly := range vallll {
			grownPoly, _ := geo.GrowPolygon(normalPoly, 0.001)
			grownPolys = append(grownPolys, grownPoly)
		}*/
	//grownPoly, err := geo.GrowPolygon(vallll[0], 0)
	//vallll = append(vallll, grownPoly)

	/*
		hello := fffcccc.Features[0].Geometry.Polygon

		sup := [][][]float64{hello[0][:len(hello[0])-1]}
		fmt.Printf("normal: %v\n", sup)
		yoyo, err := geo.GrowPolygon2(sup, 0.001)
		if err != nil {
			fmt.Printf("err: %v\n", err)
		}
		fmt.Printf("grown: %v\n", sup)

		gm := geojson.NewPolygonFeature(yoyo)
		ft := geojson.NewFeatureCollection()
		ft.AddFeature(gm)
	*/
	grownFeatureCollection := geo.PolygonToFeatureCollection(grownPolys)
	grownFCmarshalled, err := grownFeatureCollection.MarshalJSON() //manually marshalling, to detect errors

	if err != nil {
		fmt.Errorf("couldn't marshal grownPolys: %v\n", err)
		c.JSON(200, gin.H{
			"cells": "failleed",
		})
		return
	}
	//fmt.Printf("Returning: %s\n", grownFCmarshalled)
	c.JSON(200, gin.H{
		"cells": grownFCmarshalled,
	})
}

// CheckIntersection checks intersection of geoJSON geometries with a point and with a circle
func (u GeometryController) CheckIntersection(c *gin.Context) {
	lat, err := strconv.ParseFloat(c.PostForm("lat"), 64)
	lng, err := strconv.ParseFloat(c.PostForm("lng"), 64)
	//radius, err := strconv.ParseFloat(c.PostForm("radius"), 64)

	gJSON := []byte(c.PostForm("geojson"))
	//maxLevel, err := strconv.Atoi(c.PostForm("max_level_geojson"))
	//minLevel, err := strconv.Atoi(c.PostForm("min_level_geojson"))
	//maxLevelCircle, err := strconv.Atoi(c.PostForm("max_level_circle"))

	fs, err := geo.DecodeGeoJSON(gJSON)

	if err != nil {
		c.JSON(400, gin.H{
			"error": err.Error(),
		})
		return
	}

	ll := s2.LatLngFromDegrees(lat, lng)
	intersectionPoint := s2.PointFromLatLng(ll)

	intersectionsFound := 0

	for _, f := range fs {
		if f.Geometry.IsPolygon() {
			for _, p := range f.Geometry.Polygon {
				polygon := geo.PointsToPolygon(p)

				if polygon.ContainsPoint(intersectionPoint) {
					intersectionsFound += 1
					fmt.Println("Intersection!")
				} else {
					fmt.Println("No intersection")
				}
			}
		}
	}

	c.JSON(200, gin.H{
		"intersections": intersectionsFound,
	})
}
