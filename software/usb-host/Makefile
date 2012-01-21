
CXXFLAGS=-Wall

all : microorb factory-test

microorb : main.o microorb.o
	$(CXX) $^ -o $@ -lusb

factory-test : factory-test.cc microorb.o
	$(CXX) $^ -o $@ -lusb

clean:
	rm -f microorb main.o microorb.o
