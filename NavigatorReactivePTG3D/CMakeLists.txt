PROJECT(NavigatorReactivePTG3D)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED base gui opengl slam maps reactivenav)

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CReacNavPTGApp3D.cpp
	CReacNavPTGApp3D.h
	reactnav3D_OM.cpp
	reactnav3D_OM.h
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} )

