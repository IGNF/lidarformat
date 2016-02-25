[![Build Status](https://travis-ci.org/IGNF/lidarformat.svg?branch=master)](https://travis-ci.org/IGNF/lidarformat)

LidarFormat

LidarFormat is an open source library for efficiently handling 3D point 
clouds with a variable number of attributes at runtime.

Homepage: 

	https://github.com/IGNF/lidarformat
	
Copyright:
	
	Institut Geographique National & CEMAGREF (2009)

Original Author: 

	Adrien Chauve
	
Contributors:

	Nicolas David, Olivier Tournaire, Bruno Vallet, Mathieu Brédif



    LidarFormat is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published
    by the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    LidarFormat is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public 
    License along with LidarFormat.  If not, see <http://www.gnu.org/licenses/>.
    
Installation:

LidarFormat uses `cmake`. To build under `Linux`, go to  the root of the repository and type:
```
mkdir build
cd build
cmake ..
make
```
If everything went well, you can install the library with
```
sudo make install
```
which will make your library available to other `C++` project.
To use LidarFormat in another cmake project, add
```
find_package(LidarFormat)
include_directories(${LidarFormat_INCLUDE_DIRS})
add_library(myLib ...)
target_link_libraries(myLib LidarFormat ...)
```
to your `CMakeLists.txt`
