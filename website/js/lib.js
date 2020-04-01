
let map, geoJsonLayer, geoJsonBufferedLayer, geoJsonEditor, geoJsonBuffEditor, marker, circleLayer;
let polygon_cells = [];
let circle_cells = [];

let getBuffedUrl = '/get_buffed';
let checkPointUrl = '/check_intersection';
let self, app;


let s2_geojson = {
    Init: function() {
        self = this;
        this.initEditor();
        this.initBuffEditor();
        this.initMap();
        this.initMapControls();
        this.bindEvents();
        return this;
    },
    initEditor: function() {
        geoJsonEditor = CodeMirror.fromTextArea(document.getElementById('geoJsonInput'), {
            mode: "javascript",
            theme: "default",
            lineNumbers: true,
        });
    },
    initBuffEditor: function() {
        geoJsonBuffEditor = CodeMirror.fromTextArea(document.getElementById('geoJsonBuffInput'), {
            mode: "javascript",
            theme: "default",
            lineNumbers: true,
        });
    },
    initMap : function() {
        map = L.map('map').setView([51.505, -0.09], 13);

        L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=pk.eyJ1IjoibWFwYm94IiwiYSI6ImNpejY4NXVycTA2emYycXBndHRqcmZ3N3gifQ.rJcFIG214AriISLbB6B5aw', {
            maxZoom: 18,
            attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors, ' +
                '<a href="https://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, ' +
                'Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
            id: 'mapbox/streets-v11',
            tileSize: 512,
            zoomOffset: -1,
            measureControl: true
        }).addTo(map);

        // 'file' is a geojson layer
        L.geoJSON(geoJsonLayer, {
            onEachFeature: colorlayer,
            style: {
                color: "#00008c",
                opacity: 0.6,
                fillColor: '#333333',
                fillOpacity: 0
            }
            }).addTo(map);
            function colorlayer(feature, layer) {
            layer.on('mouseover', function (e) {
                layer.setStyle({
                    fillOpacity: 0.4
                });
            });
            layer.on('mouseout', function (e) {
                layer.setStyle({
                    fillOpacity: 0
                });
            });
        }
    },
    initMapControls : function() {
        map.addControl(
            new L.Control.Draw({
            draw: {
                circle: false,
                polyline: false,
                rectangle: false,
                circlemarker: false,
            },
            edit: {
                featureGroup: new L.FeatureGroup()
            }
        }));

        map.addControl(
            new L.control.measure({
                position: 'topleft'
            }));
    },
    bindEvents : function() {
        let buffer_in_meters_slider = document.getElementById("buffer_in_meters");
        let buffer_in_meters_value = document.getElementById("buffer_in_meters_value");
        buffer_in_meters_value.value = buffer_in_meters_slider.value;
        buffer_in_meters_slider.oninput = function() {
            buffer_in_meters_value.value = this.value;
            //Add growPoly
            self.RequestBuffed();
        };

        let geoJsonInput = document.getElementById("geoJsonInput");
        geoJsonInput.oninput = function() {
            self.GeoJsonToMap();
        };

        let geoJsonBuffInput = document.getElementById("geoJsonBuffInput");
        geoJsonBuffInput.oninput = function() { //remove oninput ability when debug done
            self.GeoJsonBuffToMap();
        };

        document.getElementById("lat").oninput = function() {
            self.checkPointIntersection();
        };
        document.getElementById("lng").oninput = function() {
            self.checkPointIntersection();
        };
        document.getElementById("buffer_in_meters_value").onchange = function() {
            if (buffer_in_meters_value.value>30000)
                buffer_in_meters_value.value=30000
            if (buffer_in_meters_value.value<0)
                buffer_in_meters_value.value=0
            buffer_in_meters_slider.value = buffer_in_meters_value.value;
            self.RequestBuffed();
        };

        map.on('click', this.onMapClick);
        geoJsonEditor.on("change", self.onGeoJsonChange);
        geoJsonBuffEditor.on("change", self.GeoJsonBuffToMap);

        map.on('draw:drawstart', function () {
            map.off('click', self.onMapClick);
            geoJsonEditor.off("change", self.onGeoJsonChange);
        });
        map.on('draw:drawstop', function () {
            map.on('click', self.onMapClick);
            geoJsonEditor.on("change", self.onGeoJsonChange);
        });
        map.on(L.Draw.Event.CREATED, self.onDrawCreated);
    },
    onMapClick : function(e) {

        let lat = e.latlng.lat;
        let lng = e.latlng.lng;

        document.getElementById("lat").value = lat;
        document.getElementById("lng").value = lng;

        self.checkPointIntersection();
    },
    checkPointIntersection: function() {
        let lat = document.getElementById("lat").value;
        let lng = document.getElementById("lng").value;

        if (marker) {
            map.removeLayer(marker)
        }

        marker = L.marker([lat, lng]).addTo(map);

        let geoJSONBuffed = geoJsonBuffEditor.getValue();
        if (geoJSONBuffed !== '') {
            let params = "lat=" + lat + "&lng=" + lng + "&geojson=" + geoJSONBuffed.trim();
            self.postRequest(params, checkPointUrl, function (response) {
                let res = JSON.parse(response);
                let intersectsPointElem = document.getElementById("intersects_with_point");
                intersectsPointElem.className = "";

                if (res.intersections>0)
                {
                    intersectsPointElem.classList.add("success");
                    if (res.intersections==1)
                        intersectsPointElem.innerHTML = "Marker is inside polygon";
                    else
                        intersectsPointElem.innerHTML = "Marker is inside " + res.intersections +" polygons";
                }
                else
                {
                    intersectsPointElem.classList.add("error");
                    intersectsPointElem.innerHTML = "No polygons contain the marker";
                }
            });
        }
    },
    onDrawCreated : function(e) {
        if (geoJsonLayer) {
            map.removeLayer(geoJsonLayer);
        }
        document.getElementById("geoJsonInput").value = '';
        geoJsonEditor.setValue("");

        self.removePolygonCells();

        let type = e.layerType;
        if (type === 'polygon' || type === 'marker') {
            geoJsonLayer = e.layer;
            let json = {
                "type": "FeatureCollection",
                "features": [geoJsonLayer.toGeoJSON(14)]
            };
            geoJsonEditor.setValue(JSON.stringify(json,null, 2));
            map.addLayer(geoJsonLayer);
            if (type === 'polygon') {
                map.fitBounds(geoJsonLayer.getBounds());
            }
            //Add growPoly
            self.RequestBuffed();
        }
    },
    onGeoJsonChange : function() {
        self.GeoJsonToMap();
    },
    onGeoJsonBuffChange : function() {
        self.GeoJsonBuffToMap();
    },
    GeoJsonToMap : function() {
        let v = geoJsonEditor.getValue();

        if (geoJsonLayer) {
            map.removeLayer(geoJsonLayer)
        }

        //self.regionCover();
        geoJsonLayer = L.geoJSON(JSON.parse(v), {}).addTo(map);
        map.fitBounds(geoJsonLayer.getBounds());

        //Add growPoly
        self.RequestBuffed();
    },
    GeoJsonBuffToMap : function() {
        let v = geoJsonBuffEditor.getValue();

        if (geoJsonBufferedLayer) {
            map.removeLayer(geoJsonBufferedLayer)
        }

        if (marker) {
            map.removeLayer(marker)
        }
        document.getElementById("intersects_with_point").innerHTML="";

        geoJsonBufferedLayer = L.geoJSON(JSON.parse(v), {color: 'red'}).addTo(map);
        //map.fitBounds(geoJsonBufferedLayer.getBounds());

        /*
        self.removePolygonCells();

        let max_level_geojson = document.getElementById("max_level_geojson").value;
        let min_level_geojson = document.getElementById("min_level_geojson").value;

        let params = "max_level_geojson=" + max_level_geojson + "&min_level_geojson=" + min_level_geojson  + "&geojson=" + geoJsonEditor.getValue().trim();
        self.postRequest(params, coverUrl, function (response) {
            let res = JSON.parse(response);
            document.getElementById("cell_tokens").value = res.cell_tokens;
            let s2cells = res.cells;
            for (let i = 0; i < s2cells.length; i++) {
                polygon_cells.push(L.polygon(s2cells[i], {color: 'red'}).addTo(map));
            }
        });*/
    },
    RequestBuffed : function() {
        let buffer_in_meters = document.getElementById("buffer_in_meters").value;
        let params = "geojson=" + geoJsonEditor.getValue().trim() +"&buffer_meters="+buffer_in_meters;
        self.postRequest(params, getBuffedUrl, function (response) {
            let res = JSON.parse(response);
            geoJsonBuffEditor.setValue(JSON.stringify(JSON.parse(atob(res.cells)),null, 2));
            /*
            document.getElementById("cell_tokens").value = res.cells;
            let s2cells = res.cells;
            for (let i = 0; i < s2cells.length; i++) {
                polygon_cells.push(L.polygon(s2cells[i], {color: 'red'}).addTo(map));
            }*/
        });
    },
    postRequest : function(params, url, callback) {
        let xmlHttp = new XMLHttpRequest();
        xmlHttp.onreadystatechange = function () {
            if (xmlHttp.readyState === 4 && xmlHttp.status === 200) {
                callback(xmlHttp.responseText);
            }
        };
        xmlHttp.open(
            "POST",
            url,
            true);
        xmlHttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
        xmlHttp.send(params);
    },
    removePolygonCells : function () {
        for (var i = 0; i < polygon_cells.length; i++) {
            map.removeLayer(polygon_cells[i]);
        }
        polygon_cells = [];
    },
    removeCircleCells : function () {
        for (var i = 0; i < circle_cells.length; i++) {
            map.removeLayer(circle_cells[i]);
        }
        circle_cells = [];
    }
};

document.addEventListener("DOMContentLoaded", function() {
    app = s2_geojson.Init();
});


