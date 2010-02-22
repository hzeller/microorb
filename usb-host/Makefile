
CXXFLAGS=-Wall

orb : orb.o microorb.o
	$(CXX) -lusb $^ -o $@

clean:
	rm -f orb orb.o microorb.o
