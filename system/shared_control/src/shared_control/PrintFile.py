import os.path
import numpy as np

class PrintFile:
    """
    Class PrintFile \n
    Args:
        predictor_type: type of predictor
        index_test: index of the test
        name_user_test: name of the user
        input_type: name of user input control type
    """
    def __init__(self, predictor_type, index_test, name_user_test, input_type):
        self._my_path = os.path.abspath(os.path.dirname(__file__))
        self._path = os.path.split(self._my_path)
        self._new_path = os.path.split(self._path[0])
        self._final_path = None
        self._prefix_name = None
        self._file = None
        
        #Create directory
        self.createDir(predictor_type, name_user_test, input_type)
        
        #Create and open file
        self.createFile(predictor_type, index_test, name_user_test, input_type)


    def createFile(self, predictor_type, index_test, name_user_test, input_type):
        """
        Create and open file file \n
        Args:
            predictor_type: type of predictor
            index_test: index of the test
            name_user_test: name of the user
            input_type: name of user input control type
        """
        len_predictor = len(predictor_type)
        prefix_predictor = None
        if(len_predictor < 3):
            prefix_predictor = predictor_type[0:len_predictor]
        else:
            prefix_predictor = predictor_type[0:3]

        prefix_user = None
        len_user = len(name_user_test)
        if(len_user < 5):
            prefix_user = name_user_test[0:len_user]
        else:
            prefix_user = name_user_test[0:5]

        prefix_input = None
        len_input = len(input_type)
        if(len_input < 3):
            prefix_input = input_type[0:len_input]
        else:
            prefix_input = input_type[0:3]
        
        self._prefix_name = prefix_predictor + "_" + prefix_user + "_" + prefix_input

        suffix_name = "_test_" + str(index_test) + ".txt"

        #Create name of the file as suffix_name + prefix_name
        final_doc_name = self._prefix_name + suffix_name
        
        #String with path of the file
        final_path_doc_name = self._final_path + "/" + final_doc_name

        #Open
        self._file = open(final_path_doc_name, 'w+')


    def createDir(self, predictor_type, name_user_test, input_type):
        """
        Create directory \n
        Args:
            predictor_type: type of predictor
            index_test: index of the test
            input_type: name of user input control type
        """
        final_path = os.path.join(self._new_path[0], "stat")
        if(not os.path.exists(final_path)):
            os.mkdir(final_path)

        final_path = os.path.join(final_path, name_user_test)
        if(not os.path.exists(final_path)):
            os.mkdir(final_path)

        final_path = os.path.join(final_path, predictor_type)
        if(not os.path.exists(final_path)):
            os.mkdir(final_path)
    

        self._final_path = os.path.join(final_path, input_type)
        if(not os.path.exists(self._final_path)):
            os.mkdir(self._final_path) 

    
    def write(self, info):
        """
        Write info \n
        Args: 
            info: info to write in the file
        """
        string = str(info) + "\n"
        self._file.write(string)

    
    def write_with_title(self, info, title):
        """
        Write info with title \n
        Args:
            info: info to write in the file
            title: title of the info
        """
        arr = None
        if(isinstance(info, np.ndarray)):
            arr = str(info[0]) + " " + str(info[1]) + " " + str(info[2])
        elif(isinstance(info, list) or isinstance(info, tuple)):
            arr = ' '.join(map(str, info)) 
        else:
            arr = info
        string = str(title) + ": " + str(arr) + "\n"
        self._file.write(string)

    
    def end_block(self):
        """
        Write === to divide two blocks
        """
        string = '===\n'
        self._file.write(string)

    
    def close(self):
        self._file.flush()
        self._file.close()

    
    def newFile(self, index_test):
        """
        Create and open a new file \n
        Args:
            index_test: index of the test to save
        """
        suffix_name = "_test_" + str(index_test) + ".txt"
        
        #Create name of the file as suffix_name + prefix_name
        final_doc_name = self._prefix_name + suffix_name

        #String with path of the file
        final_path_doc_name = self._final_path + "/" + final_doc_name

        #Open
        self._file = open(final_path_doc_name, 'w+')

