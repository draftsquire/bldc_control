from matplotlib import pyplot as plt
import numpy as np
import scipy

def main():
    filename = "log_p_0_4.csv"
    data = np.genfromtxt(filename, delimiter=";", skip_header=1, dtype=float)
    timestamps = data[:, 0]
    #converting UNIX timestamp in seconds
    timestamps = (timestamps - timestamps[0])/1000.
    desired_pos = data[:, 1]
    error = data[:, 2]
    position = data[:, 3]
    speed_rpm = data[:, 4]
    offset = 0
    for i, val in enumerate(position):
        if val > 0:
            print(i)
            offset = i
            break
    timestamps = timestamps[:-offset]
    plt.subplot(2, 1, 2)
    # print(position)
    plt.plot(timestamps, position[offset:], label="$x(t)$")
    plt.plot(timestamps, desired_pos[offset:], label="$x^*$")
    plt.legend(loc="upper right", frameon=True)
    plt.grid()
    plt.minorticks_on()
    plt.grid(which='minor', linewidth='0.1', color='black')
    plt.xlabel("Время, с")
    plt.ylabel("мм")

    # plt.subplot(2, 2, 2)
    # plt.plot(timestamps, desired_pos)
    # plt.grid()
    # plt.minorticks_on()
    # plt.grid(which='minor', linewidth='0.1', color='black')
    # plt.xlabel("Время, с")
    # plt.ylabel("$x^*$, мм")

    plt.subplot(2, 2, 1)
    plt.plot(timestamps, error[offset:], color="red")
    plt.grid()
    plt.minorticks_on()
    plt.grid(which='minor', linewidth='0.1', color='black')
    plt.xlabel("Время, с")
    plt.ylabel("$e(t)$, мм")

    plt.subplot(2, 2, 2)
    # speed_rpm = scipy.signal.medfilt(speed_rpm, kernel_size=51)
    plt.plot(timestamps, speed_rpm[offset:], color="orange")
    plt.grid()
    plt.minorticks_on()
    plt.grid(which='minor', linewidth='0.1', color='black')
    plt.xlabel("Время, с")
    plt.ylabel("$\omega$, об / мин")

    plt.suptitle("P = 0.4")
    # plt.suptitle("P = 1.4, D = 0.2")

    plt.show()

    # plt.plot(data[:1000000])
    # plt.show()



    # Отбираем нужное количество элементов, соответствующее одному тайм-слоту + ещё чут чут
    # data = data[(slots_n-1)*int(60 * fs / 2250):slots_n*int(60 * fs / 2250)+500]
    # time = time[(slots_n-1)*int(60 * fs / 2250):slots_n*int(60 * fs / 2250)+500]
    # databits = databits[(slots_n-1)*int(60 * fs / 2250):slots_n*int(60 * fs / 2250)+500]
    # data = data[28000:28000+2000]
    # time = time[28000:28000+2000]
    # databits = databits[28000:28000+2000]
    # plt.figure("2")
    # plt.plot(time, np.abs(data), linewidth=0.5)
    # plt.xlabel("Time")
    # plt.ylabel("Amplitude")
    # plt.title(os.path.basename("../../materials/AIS_IQ.wav")+" Original signal")
    # plt.grid()
    # plt.show()
    #
    # plt.figure("3")
    # plt.plot(time, np.abs(data_baseband), linewidth=0.5)
    # plt.xlabel("Time")
    # plt.ylabel("Amplitude")
    # plt.title(os.path.basename("../../materials/AIS_IQ.wav")+" Baseband signal")
    # plt.grid()
    # plt.show()



    # # Отмечаем пики, обнаруженные CFAR
    # msg_markers = sync.sync_branches[i].msg_peaks_pos
    # # if len(msg_markers) != 0:
    # # plt.subplot(len(fd_list_sync[0:10]), 1, i + 1)
    # markers = sync.sync_branches[i].peaks_pos
    # corr = np.abs(sync.sync_branches[i].corr)
    # plt.plot( corr, "-b.", markevery=markers, mfc="black", mec="black", linewidth=1, markersize=15)
    # # Отмечаем наилучшие пики для каждого сообщения
    # best_markers = sync.sync_branches[i].best_peaks_pos
    # plt.plot( corr, "-b*", markevery=best_markers, mfc="green", mec="green",
    #           linewidth=1, markersize=18)
    # plt.plot( corr, "-bX", markevery=msg_markers, mfc="red", mec="red",
    #           linewidth=1, markersize=18)
    # plt.ylabel("Fi = " + str(int(fd)) + " Hz")
    # plt.grid(color='grey', linestyle='--', linewidth=1)
    # plt.show()



        # filename = "signals/AIS_discriminator_%0.2d.wav" % i
        # disc_data = (demodulator.b*std).astype(np.int16)
        # wav.write(filename, fs, disc_data)
        # fs_hat, data_hat = wav.read(filename)
        # print(filename+" Sample rate:", fs_hat)
        # print(filename+" Data", data_hat)

        # plt.vlines(x=[time[msg_start - 24*5], time[msg_start - 5]], ymin=-1, ymax=1, colors='red', ls='--', lw=0.8, label='Preamble borders')



        # plt.subplot(2, 1, 1)
        # plt.plot(demodulator.b[msg_start - 24 * sym_len: msg_start + 256*sym_len], linewidth=1)
        # plt.ylabel("AMP")
        # plt.xlabel("s")
        # plt.title("Demodulated signal")
        # plt.grid(color='grey', linestyle='--', linewidth=1)
        #
        # plt.subplot(2, 1, 2)
        # plt.plot(ind, "g-", linewidth=0.5)
        # plt.ylabel("Integrate & dump")
        # plt.xlabel("s")
        # plt.title("Integrate & dump")
        # plt.grid(color='grey', linestyle='--', linewidth=1)
        # plt.show()
        # plt.show()



        # with open('data.log', 'a') as the_file:
        #     the_file.write("{\"data\":\"" + data + "\",\"checksum\":\"" + sfcs + "\"\n")
        # print("\n")

        # print(i, " message BER is ", get_ber(data_nrzi[i], nrzi))
        # # Смотрим на сообщение с компенсацией эффекта Допплера
        # ind = demodulator.integrate_and_dump_filter(0, len(ais_signal.nrzi))
        # plt.plot(ind, "g-", linewidth=0.5)
        # plt.ylabel("Integrate & dump")
        # plt.xlabel("s")
        # plt.title("Integrate & dump")
        # plt.grid(color='grey', linestyle='--', linewidth=1)
        # plt.show()

    pass


if __name__ == '__main__':
    main()