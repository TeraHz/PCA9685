CXXFLAGS := -fPIC -O3 -g -Wall -Werror
CXX := arm-unknown-linux-gnueabi-g++
MAJOR := 0
MINOR := 1
NAME := PCA9685
VERSION := $(MAJOR).$(MINOR)

lib: lib$(NAME).so.$(VERSION) lib$(NAME).so

lib$(NAME).so: lib$(NAME).so.$(VERSION)
	ldconfig -v -n .
	ln -s lib$(NAME).so.$(MAJOR) lib$(NAME).so

lib$(NAME).so.$(VERSION): $(NAME).o
	$(CXX) -shared -Wl,-soname,lib$(NAME).so.$(MAJOR) $^ -o $@

clean:
	$(RM) $(NAME)_test *.o *.so*

