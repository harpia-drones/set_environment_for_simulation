SHELL := /bin/bash

name :=

.PHONY: create create-bringup create-interfaces

################## create a ros2 package for nodes ##################
create:

# verify is the name was defined
ifdef name
# if name is defined, then create the package
	@ros2 pkg create $(name) --build-type ament_python --dependencies rclpy --license Apache-2.0
else
	@echo "The package name is missing."
	@echo 'Example: make create name="my_pkg"'
endif 

# validate the changes
	@cd .. && colcon build --packages-select $(name) 
	@cd .. && source ~/.bashrc
	@chmod -R a+rwx $(name)

################## create a ros2 package for bringup ##################
create-bringup:
ifdef name
	@ros2 pkg create $(name)_bringup --license Apache-2.0
else
	@echo "The package name is missing."
	@echo 'Example: make create-bringup name="my_pkg"'
endif

# remove the unused directories
	@cd $(name)_bringup && rm -rf install/ src/ include/
	@cd $(name)_bringup && mkdir launch  

# copy and edit the CMakeList.txt file
	@cp dependencies/bringup/CMakeLists.txt.txt $(name)_bringup/CMakeLists.txt
	@sed -i '2s/.*/project($(name)_bringup)/' $(name)_bringup/CMakeLists.txt

# copy and edit the package.xml file
	@cp dependencies/bringup/package.xml.txt $(name)_bringup/package.xml
	@sed -i '4s/.*/  <name>$(name)_bringup<\/name>/' $(name)_bringup/package.xml

# validate the changes
	@cd .. && colcon build --packages-select $(name)_bringup
	@cd .. && source ~/.bashrc
	@chmod -R a+rwx $(name)_bringup/ 

################## create a ros2 package for interfaces ##################
create-interfaces:

# verify is the name was defined
ifdef name
# if name is defined, then create the package
	@ros2 pkg create $(name)_interfaces --license Apache-2.0
else
	@echo 'Example: make create-interfaces name="my_pkg"'
	@echo "The package name is missing."
endif

# remove the unused directories
	@cd $(name)_interfaces && rm -rf install/ src/ include/
	@cd $(name)_interfaces && mkdir msg srv action

# copy and edit the CMakeList.txt file
	@cp dependencies/interfaces/CMakeLists.txt.txt $(name)_interfaces/CMakeLists.txt
	@sed -i '2s/.*/project($(name)_interfaces)/' $(name)_interfaces/CMakeLists.txt

# copy and edit the package.xml file
	@cp dependencies/interfaces/package.xml.txt $(name)_interfaces/package.xml
	@sed -i '4s/.*/  <name>$(name)_interfaces<\/name>/' $(name)_interfaces/package.xml

# validate the changes
	@cd .. && colcon build --packages-select $(name)_interfaces 
	@cd .. && source ~/.bashrc
	@chmod -R a+rwx $(name)_interfaces/
