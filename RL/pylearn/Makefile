INCLUDES =  -I../../../include/ -I/usr/include/python2.7 `pkg-config --cflags ode`
LIBS = /usr/local/lib/libdrawstuff.a  -L/usr/lib/python2.7/config `pkg-config --libs ode`
OPTS = -g -O0 -lglut -lGL -lGLU -lX11 -lXmu -lXi -lm -lboost_python -lpython2.7
CXX = g++

server: server.cpp
	${CXX} ${INCLUDES} -o $@ $< ${LIBS} ${OPTS}

client_test: client_test.cpp tcp.cpp
	${CXX} ${INCLUDES} -o $@  $? ${LIBS} ${OPTS}

boostnumpy_test: boostnumpy_test.cpp
	${CXX} ${INCLUDES} -o $@ $< ${LIBS} ${OPTS}

numpy: numpy.cpp
	${CXX} -fPIC ${INCLUDES} -c $< 
	${CXX} -fPIC ${INCLUDES} -o hello.so $< ${LIBS} ${OPTS} -shared 

test: test.cpp
	${CXX} ${INCLUDES} -o $@ $< ${LIBS} ${OPTS}

test_tac: test_tac.cpp
	${CXX} ${INCLUDES} -o $@ $< ${LIBS} ${OPTS}

