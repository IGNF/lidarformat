# Locate Xsd from code synthesis include paths and binary
# Xsd can be found at http://codesynthesis.com/products/xsd/
# Written by Frederic Heem, frederic.heem _at_ telsey.it

# This module defines
# XSD_INCLUDE_DIR, where to find elements.hxx, etc.
# XSD_EXECUTABLE, where is the xsd compiler
# XSD_FOUND, If false, don't try to use xsd

FIND_PATH(XSD_INCLUDE_DIR cxx/parser/elements.hxx
  "C:/Program Files/CodeSynthesis XSD 3.2/include/xsd"
  $ENV{XSDDIR}/include
  $ENV{CODESYNTH}/include/xsd
  /usr/local/include/xsd
  /usr/include/xsd
)

FIND_PROGRAM(XSD_EXECUTABLE 
  NAMES 
    xsd
  PATHS
	"C:/Program Files/CodeSynthesis XSD 3.2/bin"
    "[HKEY_CURRENT_USER\\xsd\\bin"
    $ENV{XSDDIR}/bin 
    /usr/local/bin
    /usr/bin
)

# if the include and the program are found then we have it
IF(XSD_INCLUDE_DIR)
  IF(XSD_EXECUTABLE)
    SET( XSD_FOUND "YES" )
  ENDIF(XSD_EXECUTABLE)
ENDIF(XSD_INCLUDE_DIR)

MARK_AS_ADVANCED(
  XSD_INCLUDE_DIR
  XSD_EXECUTABLE
) 
