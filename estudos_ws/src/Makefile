SHELL := /bin/bash

name :=
urdf_path :=

.PHONY: create create-bringup create-interfaces

#####################################################
# create a ros2 package for nodes
#####################################################

create:

# verify is the name was defined
ifdef name
# if name is defined, then create the package
	@ros2 pkg create $(name) --build-type ament_python --dependencies rclpy --license Apache-2.0

# validate the changes
	@cd .. && colcon build --packages-select $(name) 
	@cd .. && source ~/.bashrc
	@chmod -R a+rwx $(name)

# if name is not defined, show an error message
else
	@echo "The package name is missing."
	@echo 'Example: make create name="my_pkg"'
endif 

#####################################################
# create a ros2 package for bringup launch files
#####################################################

create-bringup:

# verify is the name was defined
ifdef name
# if name is defined, then create the package
	@ros2 pkg create $(name)_bringup --license Apache-2.0

# remove the unused directories
	@cd $(name)_bringup && rm -rf src/ include/
	@cd $(name)_bringup && mkdir launch  

# copy and edit the CMakeList.txt file
	@cp dependencies/bringup/CMakeLists.txt $(name)_bringup/CMakeLists.txt
	@sed -i '2s/.*/project($(name)_bringup)/' $(name)_bringup/CMakeLists.txt

# copy and edit the package.xml file
	@cp dependencies/bringup/package.xml $(name)_bringup/package.xml
	@sed -i '4s/.*/  <name>$(name)_bringup<\/name>/' $(name)_bringup/package.xml

# copy the template node
	@cp dependencies/bringup/template.launch.py $(name)_bringup/launch/template.launch.py 

# validate the changes
	@cd .. && colcon build --packages-select $(name)_bringup
	@cd .. && source ~/.bashrc
	@chmod -R a+rwx $(name)_bringup/ 

# if name is not defined, show an error message
else
	@echo "The package name is missing."
	@echo 'Example: make create-bringup name="my_pkg"'
endif

#####################################################
# create a ros2 package for interfaces
#####################################################

create-interfaces:

# verify is the name was defined
ifdef name
# if name is defined, then create the package
	@ros2 pkg create $(name)_interfaces --license Apache-2.0

# remove the unused directories
	@cd $(name)_interfaces && rm -rf src/ include/
	@cd $(name)_interfaces && mkdir msg srv action

# copy and edit the CMakeList.txt file
	@cp dependencies/interfaces/CMakeLists.txt $(name)_interfaces/CMakeLists.txt
	@sed -i '2s/.*/project($(name)_interfaces)/' $(name)_interfaces/CMakeLists.txt

# copy and edit the package.xml file
	@cp dependencies/interfaces/package.xml $(name)_interfaces/package.xml
	@sed -i '4s/.*/  <name>$(name)_interfaces<\/name>/' $(name)_interfaces/package.xml

# validate the changes
	@cd .. && colcon build --packages-select $(name)_interfaces 
	@cd .. && source ~/.bashrc
	@chmod -R a+rwx $(name)_interfaces/

# if name is not defined, show an error message
else
	@echo 'Example: make create-interfaces name="my_pkg"'
	@echo "The package name is missing."
endif

#####################################################
# create a ros2 package for description (urdf files)
#####################################################

create-description:
# verify is the name was defined
ifdef name
# if name is defined, then create the package
	@ros2 pkg create $(name)_description --license Apache-2.0

# remove the unused directories
	@cd $(name)_description && rm -rf src/ include/
	@cd $(name)_description && mkdir urdf

ifdef urdf_path
# move the existing urdf to inside the directority n
	@mv $(name).urdf $(urdf_path)
endif 

# copy and edit the CMakeList.txt file
	@cp dependencies/description/CMakeLists.txt $(name)_description/CMakeLists.txt
	@sed -i '2s/.*/project($(name)_description)/' $(name)_description/CMakeLists.txt

# copy and edit the package.xml file
	@cp dependencies/description/package.xml $(name)_description/package.xml
	@sed -i '4s/.*/  <name>$(name)_description<\/name>/' $(name)_description/package.xml

# validate the changes
	@cd .. && colcon build --packages-select $(name)_description 
	@cd .. && source ~/.bashrc
	@chmod -R a+rwx $(name)_description/

# if name is not defined, show an error message
else
	@echo 'Example: make create-description name="my_pkg" urdf_path="/root/estudos_ws/src/my_robot.urdf" (optional)'
	@echo "The package name is missing."
endif
