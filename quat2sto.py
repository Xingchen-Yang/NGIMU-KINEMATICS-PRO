def quat2sto_single(sensor_data, header_text, file_dir, t_step, rate, sensor_ind_list):
    with open(file_dir, 'w') as f:
        # initial information to write to file
        f.write("DataRate={}\n".format(rate))
        f.write("DataType=Quaternion\n")
        f.write("version=3\n")
        f.write("OpenSimVersion=4.1\n")
        f.write("endheader\n")
        f.write(header_text)
        f.write("{}".format(t_step))
        for j in sensor_ind_list:
            #f.write("\t{},{},{},{}".format(sensor_data[j,0],sensor_data[j,1],sensor_data[j,2],sensor_data[j,3]))
            f.write("\t{},{},{},{}".format(sensor_data[j,0],sensor_data[j,1]*(-1),sensor_data[j,2]*(-1),sensor_data[j,3]*(-1)))
        f.write("\n")
        
def quat2sto(sensor_data, header_text, file_dir, t_step, rate, sensor_ind_list):
    with open(file_dir, 'w') as f:
        # initial information to write to file
        f.write("DataRate={}\n".format(rate))
        f.write("DataType=Quaternion\n")
        f.write("version=3\n")
        f.write("OpenSimVersion=4.1\n")
        f.write("endheader\n")
        f.write(header_text)
        for i in range(0, sensor_data.shape[1]):
            f.write("{}".format(t_step))
            for j in sensor_ind_list:
                #f.write("\t{},{},{},{}".format(sensor_data[j, i, 0],sensor_data[j, i, 1],sensor_data[j, i, 2],sensor_data[j, i, 3]))
                f.write("\t{},{},{},{}".format(sensor_data[j, i, 0],sensor_data[j, i, 1]*(-1),sensor_data[j, i, 2]*(-1),sensor_data[j, i, 3]*(-1)))
            f.write("\n")
            t_step = t_step + 1/rate