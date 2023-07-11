import sys, time
from matplotlib.pyplot import plot
import opensim as osim
from multiprocessing import Process, Queue, Lock
import NGtoOpenSense
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QThread
from heading_err import*
from quat2sto import*
from math import pi
from transforms3d.quaternions import quat2mat,mat2quat, qmult, qconjugate
import NG_Kinematics
import numpy as np
import pyqtgraph as pg
# a thread for calculating kinematics
class WorkThread(QThread):
    def __int__(self, flag = 1):
        super(WorkThread, self).__init__()
        self.flag = flag
    def stopKine(self):
        self.flag = 0
    def startKine(self):
        self.flag = 1
    def run(self):
        ## calculate real-time kinematics
        time_counter = 0
        modelFileName = 'calibrated_Rajagopal_2015.osim';  # The path to an input model
        #modelFileName = 'calibrated_Rajagopal_2015_lying.osim';
        model = osim.Model(modelFileName)
        coordinates = model.getCoordinateSet()
        ikReporter = osim.TableReporter()
        ikReporter.setName('ik_reporter')
        for coord in coordinates:
            ikReporter.addToReport(coord.getOutput('value'),coord.getName())
        model.addComponent(ikReporter)
        model.finalizeConnections
        while (q.qsize() > 0):  # clear the queues that may have old messages
            q.get()
        t_step = 0.0
        IMU_list = q.get()
        for IMU in IMU_list:
            IMU_quat = np.array(IMU[2: 6])
            quat_test[IMU_list.index(IMU), :] = IMU_quat
        # drift correction
        for i in range(0, num_sensors):
            quat_test[i, :] = mat2quat(np.matmul(R_gitobody0[i, :, :], quat2mat(quat_test[i, :]).T).T)
        header_text = 'time\t'
        for label in sensor_labels_list:
            header_text = header_text + '\t' + label
        header_text = header_text + '\n'
        quat2sto_single(quat_test, header_text, test_file, t_step, rate, sensor_ind_list)
        quatTable = osim.TimeSeriesTableQuaternion(test_file)
        R_HG = osim.Rotation()
        # heading correction
        R_HG.setRotationFromAngleAboutZ(-pi/2)
        osim.OpenSenseUtilities.rotateOrientationTable(quatTable, R_HG)
        # transform to Opensim coordinate system
        R_HG.setRotationFromAngleAboutX(-pi/2)
        osim.OpenSenseUtilities.rotateOrientationTable(quatTable, R_HG)
        orientationsData = osim.OpenSenseUtilities.convertQuaternionsToRotations(quatTable)
        oRefs = osim.OrientationsReference(orientationsData)
        mRefs = osim.MarkersReference()
        cRefs = osim.SimTKArrayCoordinateReference()
        visualize = True
        if visualize:
            model.setUseVisualizer(True)
        s0 = model.initSystem()
        constraint_var = 10.0
        accuracy = 0.001
        ikSolver = osim.InverseKinematicsSolver(model, mRefs, oRefs, cRefs, constraint_var)
        ikSolver.setAccuracy = accuracy
        s0.setTime(t_step)
        ikSolver.assemble(s0)
        if visualize:
            model.getVisualizer().show(s0)
            model.getVisualizer().getSimbodyVisualizer().setShowSimTime(True)
        x = []; y = []; #for ploting
        while self.flag:
            while (q.qsize() > 0):  # clear the queues that may have old messages
                q.get()
            IMU_list = q.get()
            t_step = t_step + 1/rate
            time_counter = time_counter + 1
            if visualize & (time_counter % 2 == 0):  # reduce to about 25Hz, Opensim limit
                for IMU in IMU_list:
                    IMU_quat = np.array(IMU[2: 6])
                    quat_test[IMU_list.index(IMU), :] = IMU_quat
                    # drift correction
                for i in range(0, num_sensors):
                    quat_test[i, :] = mat2quat(np.matmul(R_gitobody0[i, :, :], quat2mat(quat_test[i, :]).T).T)
                quat2sto_single(quat_test, header_text, test_file, t_step, rate, sensor_ind_list)
                quatTable = osim.TimeSeriesTableQuaternion(test_file)
                R_HG = osim.Rotation();
                # heading correction
                R_HG.setRotationFromAngleAboutZ(-pi/2);
                osim.OpenSenseUtilities.rotateOrientationTable(quatTable, R_HG)
                # transform to Opensim coordinate system
                R_HG.setRotationFromAngleAboutX(-pi/2);
                osim.OpenSenseUtilities.rotateOrientationTable(quatTable, R_HG)
                orientationsData = osim.OpenSenseUtilities.convertQuaternionsToRotations(quatTable)
                oRefs = osim.OrientationsReference(orientationsData)
                ikSolver = osim.InverseKinematicsSolver(model, mRefs, oRefs, cRefs, constraint_var)
                s0.setTime(t_step)
                ikSolver.assemble(s0)
                model.getVisualizer().show(s0)
                model.realizeReport(s0)
                #kin_list = ['pelvis_tilt', 'pelvis_list', 'pelvis_rotation', 'pelvis_tx', 'pelvis_ty', 'pelvis_tz', 'hip_flexion_r','hip_adduction_r','hip_rotation_r',	'knee_angle_r', 'knee_angle_r_beta','ankle_angle_r', 'subtalar_angle_r', 'mtp_angle_r', 'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 'knee_angle_l', 'knee_angle_l_beta', 'ankle_angle_l', 'subtalar_angle_l', 'mtp_angle_l', 'lumbar_extension', 'lumbar_bending', 'lumbar_rotation','arm_flex_r', 'arm_add_r','arm_rot_r', 'elbow_flex_r', 'pro_sup_r', 'wrist_flex_r', 'wrist_dev_r','arm_flex_l', 'arm_add_l', 'arm_rot_l', 'elbow_flex_l', 'pro_sup_l', 'wrist_flex_l', 'wrist_dev_l']
                imuLabels = ikReporter.getTable().getColumnLabels()
                idx = time_counter//2-1
                kin_step = ikReporter.getTable().getRowAtIndex(idx).getElt(0,imuLabels.index('knee_angle_r'))
                x.append(idx)
                y.append(kin_step*90/pi)
                curve.setData(x, y)
                display_len = rate/2*10
                if(idx>display_len):
                    x.pop(0); y.pop(0) 
                    curve.setPos(idx - display_len,0)
                    
def NG_start():
    #start new process for IMU data collection
    lock = Lock()
    imuProc = Process(target=NGtoOpenSense.readIMU, args=(lock, q,))
    imuProc.start()
    print("Start NGIMU Process------------------------------------")

def NG_cal():
    work.stopKine()
    print("Start NGIMU Calibration--------------------------------")
    while(q.qsize()>0): # clear the queues that may have old messages
        q.get()
    #collect calibration data
    for i in range(0, quat_cal_len):
        IMU_list = q.get()
        for IMU in IMU_list:
            IMU_quat = np.array(IMU[2: 6])
            quat_cal[IMU_list.index(IMU), i, :] = IMU_quat
    #heading correction
    #anatomy calibration
    sensortobody = np.zeros([num_sensors, 3, 3])
    # aligned with sensor_labels_list
    sensortobody[0, :, :] = np.matrix([[0, 1, 0], [0, 0, -1], [-1, 0, 0]])
    sensortobody[1, :, :] = np.matrix([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]) #right
    sensortobody[2, :, :] = np.matrix([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    # sensortobody[3, :, :] = np.matrix([[0, 0, -1], [0, -1, 0], [-1, 0, 0]]) #left
    # sensortobody[4, :, :] = np.matrix([[0, 0, -1], [0, -1, 0], [-1, 0, 0]])
    # anatomic calibration, R gi to b0 (body frame at time 0)
    gitobody0 = np.zeros([len(sensor_labels_list), 3, 3])
    quat_cal_last = np.zeros((num_sensors, 4))
    for i in range(0, num_sensors):
        gitobody0[i, :, :] = np.matmul(sensortobody[sensor_ind_list.index(i), :, :], quat2mat(quat_cal[i, -1, :]))
        R_ref = np.identity(3)
        err = heading_err(gitobody0[i, :, :], R_ref)
        R_gitobody0[i, :, :] = rotz(err)
        # R body to g
        quat_cal_last[i, :] = mat2quat(np.matmul(R_gitobody0[i, :, :], quat2mat(quat_cal[i, -1, :]).T).T)
    #save .sto file
    header_text = 'time\t'
    for label in sensor_labels_list:
        header_text = header_text + '\t' + label
    header_text = header_text + '\n'
    sensor_data = quat_cal[:, -1, :]
    t_step = 0
    quat2sto_single(sensor_data, header_text, cal_file, t_step, rate, sensor_ind_list)
    # Set variables to use
    modelFileName = 'Rajagopal_2015.osim';  # The path to an input model
    #modelFileName = 'Rajagopal_2015_lying.osim';  # For Pedro
    orientationsFileName = cal_file;  # The path to orientation data for calibration
    sensor_to_opensim_rotations = osim.Vec3(-pi/2, -pi/2, 0);  # The rotation of IMU data to the OpenSim world frame
    visulizeCalibration = False;  # Boolean to Visualize the Output model
    baseIMUName = 'pelvis_imu';
    # Instantiate an IMUPlacer object
    imuPlacer = osim.IMUPlacer();
    # Set properties for the IMUPlacer
    imuPlacer.set_model_file(modelFileName);
    imuPlacer.set_orientation_file_for_calibration(orientationsFileName);
    imuPlacer.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations);
    imuPlacer.set_base_imu_label(baseIMUName);
    # Run the IMUPlacer
    imuPlacer.run(visulizeCalibration);
    # Get the model with the calibrated IMU
    model = imuPlacer.getCalibratedModel();
    # Print the calibrated model to file.
    model.printToXML('calibrated_' + modelFileName)

def NG_run():
    work.startKine()
    work.start()
    print("Start Real-time Kinematics-----------------------------")

def NG_stop():
    exit(0)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()
    #MainWindow.setFixedSize(320, 170) 
    ui = NG_Kinematics.Ui_NG_Kinematics()
    ui.setupUi(MainWindow)
    MainWindow.show()
    ui.Start.clicked.connect(NG_start)
    ui.Calibration.clicked.connect(NG_cal)
    ui.Construction.clicked.connect(NG_run)
    ui.Stop.clicked.connect(NG_stop)
    #for plotting
    #ui.graphicsView.setWindowTitle('Kinematics')
    fig1 = ui.graphicsView.addPlot(row=0, col=0)
    fig1.setLabel("left", "Joint Angle/deg")
    fig1.setLabel("bottom", "Time/s")
    fig1.setYRange(0, 90)  
    curve = fig1.plot(pen=pg.mkPen('r', width=1))  
    work = WorkThread()
    q = Queue(5)  # maximum 5
    quat_cal_len = 100  # 50 Hz, 2 seconds of data
    num_sensors = 3
    quat_cal = np.zeros((num_sensors, quat_cal_len, 4))
    quat_test = np.zeros((num_sensors, 4))
    sensor_labels_list = ['pelvis_imu', 'femur_r_imu', 'tibia_r_imu']
    sensor_ind_list = [0, 1, 2]
    cal_file = 'cal_file.sto'
    test_file = 'test_file.sto'
    rate = 50 #sampling rate of IMU sensors
    #for heading correction
    R_gitobody0 = np.zeros([num_sensors, 3, 3])
    sys.exit(app.exec_())


