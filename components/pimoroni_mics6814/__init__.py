# Auto-loaded by ESPHome external_components
CODEOWNERS = ["@you"]

# Force-build dependencies so headers are available even when only the sensor platform is used.
# This avoids compilation failures when compiling the component without also defining the switch/button/number platforms in YAML.
DEPENDENCIES = ["i2c", "sensor", "switch", "button"]
