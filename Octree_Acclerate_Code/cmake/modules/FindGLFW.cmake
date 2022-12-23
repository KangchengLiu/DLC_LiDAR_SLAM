FIND_PATH( GLFW_INCLUDE_DIR GL/glfw.h $ENV{GLFWDIR}/include
                                      ~/Library/Frameworks/GLFW.framework/Headers
                                      /Library/Frameworks/GLFW.framework.Headers
                                      /usr/local/include
                                      /usr/local/X11R6/include
                                      /usr/X11R6/include
                                      /usr/X11/include
                                      /usr/include/X11
                                      /usr/include
                                      /opt/X11/include
                                      /opt/include )

FIND_LIBRARY( GLFW_LIBRARIES NAMES glfw PATHS $ENV{GLFWDIR}/lib
                                              $ENV{GLFWDIR}/support/msvc80/Debug
                                              $ENV{GLFWDIR}/support/msvc80/Release
                                              /usr/local/lib
                                              /usr/local/X11R6/lib
                                              /usr/X11R6/lib
                                              /usr/X11/lib
                                              /usr/lib/X11
                                              /usr/lib
                                              /opt/X11/lib
                                              /opt/lib )

SET( GLFW_FOUND "NO" )
IF ( GLFW_LIBRARIES )
  SET( GLFW_FOUND "YES" )
ENDIF( GLFW_LIBRARIES )