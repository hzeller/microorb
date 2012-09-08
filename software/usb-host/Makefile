
CXXFLAGS=-Wall

all : microorb factory-test

microorb : main.o microorb.o
	$(CXX) $^ -o $@ -lusb -lmicrohttpd

factory-test : factory-test.cc microorb.o
	$(CXX) $^ -o $@ -lusb -lncurses

clean:
	rm -f microorb factory-test main.o microorb.o factory-test.o
