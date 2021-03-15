import os.path

class PrintOnFile:
    def __init__(self):
        self.my_path = os.path.abspath(os.path.dirname(__file__))
        self.path = os.path.split(self.my_path)
        self.new_path = os.path.split(self.path[0])
        self.file_name = self.new_path[0]+"/stat_assist/output.txt"
        self.file = open(self.file_name,'w+')

    def write(self, info):
        string = str(info) + "\n"
        self.file.write(string)

    def write_with_title(self, info, title):
        string = title + ": " + str(info) + "\n"
        self.file.write(string)

    def end_block(self):
        string = '===\n'
        self.file.write(string)

    def close(self):
        self.file.flush()
        self.file.close()
