MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
OUTDIR:="$(MAKEFILE_DIR)/../"

all: example

example:
	cookiecutter "$(MAKEFILE_DIR)" --directory starling_template --no-input -f -o "$OUTDIR"
	cookiecutter "$(MAKEFILE_DIR)" --directory python_ros2_node_offboard_template --no-input -f -o "${OUTDIR}/starling_controller/starling_controller"
	cookiecutter "$(MAKEFILE_DIR)" --directory cpp_ros2_node_onboard_template --no-input -f -o "${OUTDIR}/starling_controller/starling_controller"
	cookiecutter "$(MAKEFILE_DIR)" --directory ros2_msgs_template --no-input -f -o "${OUTDIR}/starling_controller/starling_controller"

docs:
	mkdocs build -f $(MAKEFILE_DIR)/mkdocs.yml

serve-docs:
	mkdocs serve -a 0.0.0.0:8000