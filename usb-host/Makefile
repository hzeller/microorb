
CXXFLAGS=-Wall

microorb : main.o microorb.o
	$(CXX) -lusb $^ -o $@

clean:
	rm -f microorb main.o microorb.o
