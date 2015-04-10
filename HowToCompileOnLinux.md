# Introduction #

This HowTo explains how to compile the LidarFormat C++ library on Linux. Tests were made on Ubuntu Jaunty (32 and 64 bits).


# Install the dependencies #

Install the following tools or libraries:

  * Mercurial
  * CMake
  * Boost (>= 1.36)
  * Xerces
  * Code Synthesis XSD (can be downloaded [here](http://codesynthesis.com/products/xsd/download.xhtml))

_**For Ubuntu Jaunty**_, just enter this command in a terminal:

```
sudo apt-get install build-essential mercurial cmake libboost1.37-dev libxerces-c2-dev
```

and download and install the _CodeSynthesis XSD_ package ([x86](http://www.codesynthesis.com/download/xsd/3.2/linux-gnu/i686/xsd_3.2.0-1_i386.deb) or [x86-64](http://www.codesynthesis.com/download/xsd/3.2/linux-gnu/x86_64/xsd_3.2.0-1_amd64.deb)).


# Get the source #

Get the source with this command (it will create a directory "lidarformat" where you entered the command and download the source in it):

```
hg clone https://lidarformat.googlecode.com/hg/ lidarformat  
```


# Compile the library #

## Run cmake ##

Go to the lidarformat directory. If the default configuration is enough, just enter this command:

```
cmake -DCMAKE_BUILD_TYPE:STRING=Release .
```

Or, if you want to tune some parameters:

```
ccmake .
--> follow the CMake instructions, in particular:
--> launch the configure step with 'c'
--> set CMAKE_BUILD_TYPE to Release within CMake
--> tune the parameters you want
--> generate the configuration with 'g'
```

## Compile ##

Just run the following command (replace N by the number of cores or procesors you have):

```
make -jN
```

## Run the tests ##

Just enter this command and check all tests are passed:

```
make test
```

The output should be something like:
```
Running tests...
Start processing tests
Test project /home/achauve/Developpement/sources/LidarFormat/trunk
  1/  4 Testing LidarFormat_concepts             Passed
  2/  4 Testing LidarFormat_unit_tests           Passed
  3/  4 Testing LidarFormat_01_ex_basics         Passed
  4/  4 Testing LidarFormat_02_ex_filtering      Passed

100% tests passed, 0 tests failed out of 4
```

If you get errors, contact us or [report a bug](http://code.google.com/p/lidarformat/issues/entry).