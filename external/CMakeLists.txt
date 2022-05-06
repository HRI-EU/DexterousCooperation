ADD_LIBRARY(rapidjson INTERFACE)
TARGET_INCLUDE_DIRECTORIES(rapidjson INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/rapidjson/include)

ADD_LIBRARY(websocketpp INTERFACE)
TARGET_INCLUDE_DIRECTORIES(websocketpp INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/websocketpp/include)

IF (WIN32)
  TARGET_INCLUDE_DIRECTORIES(websocketpp INTERFACE 
                             ${HGR}/External/asio-1.12.1/include 
                             ${HGR}/External/boost/boost_1_59_0)
TARGET_LINK_LIBRARIES(websocketpp INTERFACE wsock32 ws2_32 Iphlpapi)
ENDIF(WIN32)