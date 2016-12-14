INCLUDES = -I ../../../include/ -I /usr/include/python2.7 `pkg-config --cflags ode`
LIBS = /usr/local/lib/libdrawstuff.a -L/usr/lib/python2.7/config `pkg-config --libs ode`
OPTS = -g -O0 -lglut -lGL -lGLU -lX11 -lXmu -lXi -lm -lboost_python -lpython2.7
CXX = g++


arm_robot3:arm_robot3.cpp world.cpp
	${CXX} ${INCLUDES} -o arm_robot3.o $< ${LIBS}  ${OPTS}

arm_client_test: arm_client_test.cpp tcp.o arm_robot3.o world.o
	${CXX} ${INCLUDES} -o $@ $? ${LIBS} ${OPTS}
