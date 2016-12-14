DATA = 1

if DATA is 1:
    input_file  = 'log'
    output_file  = 'out'
    n_state = 1
elif DATA is 2:
    input_file  = 'log2'
    output_file = 'out2'
    n_state = 2


def make_gnuplot_data(input_file, output_file):
    data = []
    with open(input_file, 'r') as read_file:
        with open(output_file, 'w') as write_file:
            for line in read_file:
                tmp = line.split(' ')
                data.append(tmp)
            print "shape of data ({0}, {1})".format(len(data), len(data[0]))

            if DATA is 1:
                data_predict_target = (data[0], data[1], data[-1])
                for val1, val2, val3 in zip(*data_predict_target):
                    write_file.write("{0} {1} {2}\n".format(val1, val2, val3))
            elif DATA is 2:
                data_predict_target = (data[0], data[1], data[2], data[-1])
                for val1, val2, val3, val4 in zip(*data_predict_target):
                    write_file.write("{0} {1} {2} {3}\n".format(val1, val2, val3, val4))

if __name__=='__main__':
    make_gnuplot_data(input_file, output_file)
