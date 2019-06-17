#!/usr/bin/env python

# Written by Mikael Glamheden
# 2019-06-05
# CarCommands is used to access a python script and execute the code that is inside of it.

class CarCommands:
    """ Reads & executes commands from file."""

    def __init__(self,filename):
        """ Initialize command object"""
        self.filename = filename
        self.commands = None

    def _read_commands(self):
        f = open(self.filename,"r")
        self.commands = f.read()
        f.close()

    def get_commands(self):
        """ Returns the code as string"""
        self._read_commands()
        return self.commands

    def execute_commands(self, globals = {}, locals = {}):
        """ Executes the code. Necessary globals or locals
            are sent as a dictionaries"""
        self._read_commands()
        exec(self.commands,globals,locals)

def test_function():
    print('test')

def main():
    filename = "/home/mikael/Desktop/commands.py"
    c = CarCommands(filename)
    print(c.get_commands())
    #print(globals())
    #print(type(globals()))
    c.execute_commands(globals())

if __name__ == '__main__':
    main()
