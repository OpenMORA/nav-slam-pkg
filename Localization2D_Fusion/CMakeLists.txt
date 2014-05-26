PROJECT(Localization2D_Fusion)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED slam gui )

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CLocalizationFusionApp.cpp
	CLocalizationFusionApp.h
	CFusionPoseEstimator.cpp
	CFusionPoseEstimator.h
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} )

