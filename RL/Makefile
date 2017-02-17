INCLUDES = -I ../include/ -I ../../include/ -I /usr/include/python2.7 -I ./include/ `pkg-config --cflags ode`
LIBS = /usr/local/lib/libdrawstuff.a -L/usr/lib/python2.7/config `pkg-config --libs ode`
OPTS = -g -O0 -Wall -Wextra  -lglut -lGL -lGLU -lX11 -lXmu -lXi -lm -lboost_python -lpython2.7 -lpthread -DdDOUBLE -std=c++11
CXX = g++


test_tac:test_tac.cpp
	${CXX} ${INCLUDES} -o $@ $< ${LIBS}  ${OPTS}

ray_test:ray_test.cpp
	${CXX} ${INCLUDES} -o $@ $< ${LIBS}  ${OPTS}

stl_test: stl_test.cpp libmesh.o
	${CXX} ${INCLUDES} -o $@ $? ${LIBS} ${OPTS}

ode_cpp_test:ode_cpp_test.cpp
	${CXX} ${INCLUDES} -o $@ $< ${LIBS}  ${OPTS} -std=c++11 -lode -ldrawstuff

arm_client_test: arm_client_test.cpp world.o robot/arm_robot3.o tcp.o nearCallback.o
	${CXX} ${INCLUDES} -o $@ $? ${LIBS} ${OPTS}

arm_client_state_learning1: arm_client_state_learning1.cpp world.o include/arm_client_state_learning1.hpp robot/arm_robot3.o tcp.o nearCallback.o
	${CXX} ${INCLUDES} -o $@ $? ${LIBS} ${OPTS}

whole_body_motion_learning1: whole_body_motion_learning1.cpp world.o include/whole_body_motion_learning1.hpp robot/baby_robot1.o tcp.o nearCallback1.o tactile_sensor.o include/tactile_sensor.hpp
	${CXX} ${INCLUDES} -o $@ $? ${LIBS} ${OPTS}

whole_body_motion_learning2: whole_body_motion_learning2.cpp world.o include/whole_body_motion_learning2.hpp robot/baby_robot2.o tcp.o nearCallback.o libmesh.o include/baby_robot2.hpp
	${CXX} ${INCLUDES} -o $@ $? ${LIBS} ${OPTS}

whole_body_motion_learning3: whole_body_motion_learning3.cpp world.o include/whole_body_motion_learning3.hpp robot/baby_robot3.o tcp.o nearCallback3.o libmesh.o include/baby_robot3.hpp
	${CXX} ${INCLUDES} -o $@ $? ${LIBS} ${OPTS}

tcp.o:tcp.cpp
	${CXX} ${INCLUDES} -c $< ${OPTS}

tactile_sensor.o:tactile_sensor.cpp
	${CXX} ${INCLUDES} -o tactile_sensor.o -c $< ${OPTS}

robot/arm_robot3.o:robot/arm_robot3.cpp
	${CXX} ${INCLUDES} -o robot/arm_robot3.o -c $< ${OPTS}

robot/arm_robot2.o:robot/arm_robot2.cpp
	${CXX} ${INCLUDES} -o robot/arm_robot2.o -c $< ${OPTS}

robot/arm_robot1.o:robot/arm_robot1.cpp
	${CXX} ${INCLUDES} -o robot/arm_robot1.o -c $< ${OPTS}

robot/rolling_robot1.o:robot/rolling_robot1.cpp
	${CXX} ${INCLUDES} -o robot/rolling_robot1.o -c $< ${OPTS}

robot/baby_robot1.o:robot/baby_robot1.cpp include/baby_robot1.hpp tactile_sensor.o
	${CXX} ${INCLUDES} -o robot/baby_robot1.o -c $< ${OPTS}

robot/baby_robot2.o:robot/baby_robot2.cpp
	${CXX} ${INCLUDES} -o robot/baby_robot2.o -c $< ${OPTS}

robot/baby_robot3.o:robot/baby_robot3.cpp
	${CXX} ${INCLUDES} -o robot/baby_robot3.o -c $< ${OPTS}

world.o:world.cpp
	${CXX} ${INCLUDES} -c $< ${OPTS}

nearCallback.o:nearCallback.cpp include/nearCallback.hpp world.o robot/baby_robot3.o robot/arm_robot1.o robot/arm_robot2.o robot/arm_robot3.o
	${CXX} ${INCLUDES} -c $<

nearCallback1.o:nearCallback.cpp include/nearCallback.hpp world.o robot/baby_robot1.o
	${CXX} ${INCLUDES} -c $< -o nearCallback1.o -D__BABY_ROBOT1__

nearCallback2.o:nearCallback.cpp include/nearCallback.hpp world.o robot/baby_robot2.o
	${CXX} ${INCLUDES} -c $< -o nearCallback2.o -D__BABY_ROBOT2__

nearCallback3.o:nearCallback.cpp include/nearCallback.hpp world.o robot/baby_robot3.o
	${CXX} ${INCLUDES} -c $< -o nearCallback3.o -D__BABY_ROBOT3__

libmesh.o:libmesh.cpp include/libmesh.hpp include/StlReader.h
	${CXX} ${INCLUDES} -c $<

test: test.cpp include/StlReader.h
	${CXX} ${INCLUDES} -o $@ $? ${LIBS} ${OPTS}
