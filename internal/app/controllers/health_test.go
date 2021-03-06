package controllers_test

import (
	"github.com/gin-gonic/gin"
	"github.com/pantrif/s2-geojson/internal/app/server"
	"github.com/stretchr/testify/assert"
	"net/http"
	"net/http/httptest"
	"testing"
)

func TestStatus(t *testing.T) {

	gin.SetMode(gin.TestMode)
	router := server.NewRouter(root)
	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/health", nil)
	router.ServeHTTP(w, req)

	assert.Equal(t, 200, w.Code)
	assert.Equal(t, "{\"status\":\"ok\"}\n", w.Body.String())
}
