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

    def _read_commands(self, id):
        """Looks after code block with the given id."""
        id = '&' + id.strip('USER') + '&'
        code_start = False
        self.commands = ''
        file = open(self.filename,"r")
        for line in file:
            if code_start:
                if '#####' in line:
                    break
                else:
                    self.commands = self.commands + line
            if id in line:
                code_start = True
        if self.commands == '':
            print('No commands with the given id')
        file.close()

    def get_commands(self, id):
        """ Returns the code as string"""
        self._read_commands(id)
        return self.commands

    def execute_commands(self, id, globals = {}, locals = {}):
        """ Executes the code. Necessary globals or locals
            are sent as a dictionaries"""
        self._read_commands(id)
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
