class Printer:
    def __init__(self, file_name, print_to_console = True):
        self.file_name = file_name
        self.print_to_console = print_to_console
        self.file = open(self.file_name, "a")

    def print(self, text, end = "\n", dont_print_to_console = False):
        # convert text to string
        text = str(text)
        # print to file with a newline
        self.file.write(text + end)
        # print to console
        if self.print_to_console:
            if not dont_print_to_console:
                print(text, end = end)
    
    def close(self):
        self.file.close()