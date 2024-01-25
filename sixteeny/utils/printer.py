class Printer:
    def __init__(self, file_name, print_to_console = True):
        self.file_name = file_name
        self.print_to_console = print_to_console
        self.log = []
        # self.file = open(self.file_name, "a")

    def print(self, text, end = "\n", dont_print_to_console = False):
        # convert text to string
        text = str(text)
        # add to log
        self.log.append(text + end)
        # print to console
        if self.print_to_console:
            if not dont_print_to_console:
                print(text, end = end)
    
    def close(self):
        self.file = open(self.file_name, "a")
        print("Writing to file...")
        # write to file
        for i,line in enumerate(self.log):
            if i//1000 == i/1000:
                print(f"Writing {i}/{len(self.log)}")
            self.file.write(line)
        # close file
        self.file.close()
        print("Done!")