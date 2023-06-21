#!/usr/bin/env python3

import os, sys
from cryptography.hazmat.primitives.ciphers import (Cipher, algorithms, modes)
from cryptography.hazmat.backends import default_backend
import numpy
import itertools
import struct
import argparse

def crc32_generate_table():
    crc32_array = [0] * 256
    crc_accum = 0
    i = 0
    while True:
        crc_accum = (i << 24) & 0xFFFFFFFF
        j = 0
        while True:
            if crc_accum & 0x80000000:
                #crc_accum = (crc_accum << 1) ^ 0x04c11db7
                crc_accum = (crc_accum << 1) ^ 0xd419cc15
            else:
                crc_accum = (crc_accum << 1)
            j += 1
            if j >= 8:
                break
        crc32_array[i] = crc_accum & 0xFFFFFFFF
        i += 1
        if i >= 256:
            break
    return crc32_array

crc32_table = crc32_generate_table()

def crc32_update(data, size):
    crc32 = 0
    j = 0

    while True:
        if j >= size:
            break
    #for byte in data:
        i = ((crc32 >> 24) ^ data[j]) & 0xFF
        crc32 = (crc32 << 8) ^ crc32_table[i]
        crc32 = crc32 & 0xFFFFFFFF
        j += 1

    return crc32

def encrypt(key, iv, plaintext):
    encryptor = Cipher(
        algorithms.AES(key),
        modes.XTS(iv),
        backend=default_backend()
    ).encryptor()
    # Encrypt the plaintext and get the associated ciphertext.
    # GCM does not require padding.
    ciphertext = encryptor.update(plaintext) + encryptor.finalize()
    return ciphertext

def decrypt(key, iv, ciphertext):
    # Construct a Cipher object, with the key, iv, and additionally the
    # GCM tag used for authenticating the message.
    decryptor = Cipher(
        algorithms.AES(key),
        modes.XTS(iv),
        backend=default_backend()
    ).decryptor()
    # Decryption gets us the authenticated plaintext.
    # If the tag does not match an InvalidTag exception will be raised.
    return decryptor.update(ciphertext) + decryptor.finalize()

def word_align(fn) :
    with open(fn, 'rb') as f :
        while True :
            trunk = f.read(4)
            if trunk :
                n = len(trunk)
                yield trunk + b'\x00' * (4-n)
            else :
                break

def gen_1k_stream(din) :
    data = bytes()
    for d in din :
        data = data + d
        if len(data) == 2**12 :
            yield data
            data = bytes()
    yield data

def combine_crc(din, fd = None, fc = None) :
    data = bytes()
    crcs = list()
    for d in din :
        if len(d) != 0:
            crcs.append(crc32_update(d, len(d)))
            data = data + d
    crc_bytes = bytes()
    for c in crcs :
        crc_bytes = crc_bytes + c.to_bytes(4, 'little', signed=False)
    if fd != None :
        with open(fd, 'wb') as f :
            f.write(data)
    if fc != None :
        with open(fc, 'wb') as f :
            f.write(crc_bytes)
    return data + crc_bytes

def gen_alps_hw_crc(din) :
    crcs = list()
    crc_bytes = bytes()
    for d in din :
            if len(d) != 0:
                crcs.append(crc32_update(d, len(d)))
    for c in crcs :
        crc_bytes = crc_bytes + c.to_bytes(4, 'little', signed=False)
    return crc_bytes

def data_append(din):
    data = bytes()
    for d in din:
        if len(d) != 0:
            data = data + d
    return data


def grouper(iterable, n, fillvalue=None):
    "Collect data into fixed-length chunks or blocks"
    # grouper('ABCDEFG', 3, 'x') --> ABC DEF Gxx"
    args = [iter(iterable)] * n
    return itertools.zip_longest(*args, fillvalue=fillvalue)

def shuffle_word_order(din) :
    for g in grouper(din, 16, 0) :
        g = bytes(g)
        data = g[12:] + g[8:12] + g[4:8] + g[0:4]
        yield data

def shuffle_word_order1(din) :
    for g in din :
        for gg in grouper(g, 16, b'\x00') :
            gg = bytes(gg)
            data = gg[12:] + gg[8:12] + gg[4:8] + gg[0:4]
            yield data

def shuffle_byte_rev_order(din) :
    for g in grouper(din, 16, 0) :
        g = g[::-1]
        data = bytes(g)
        yield data

def shuffle_byte_rev_order1(din) :
    for g in din :
        for gg in grouper(g, 16, b'\x00') :
            gg = gg[::-1]
            data = bytes(gg)
            yield data

def encrypt_stream(din, key, iv) :
    for g in grouper(din, 4, b'\x00' * 16) :
        txt = b''.join(g)
        yield encrypt(key, iv, txt)

def gen_header(filename, boot_flag, xip_flag):
    portion = os.path.splitext(filename)
    ininame = portion[0] + ".ini"
    elfname = portion[0] + ".elf"

    file0 = open(ininame, 'r')
    ini_info = file0.readlines()
    file0.close()

    load_addr = 0
    header_pos = 0
    header_size = 512
    enterpoint = 0
    ram_base = 0

    line_no = 0
    toolchain = 0
    t_base = 0

    pload_size_pos = 20
    ram_base_pos = 24
    exec_offset_pos = 28

    flash_mmap_firmware_base = 0x300000

    if system_boot_stage == 3:
        if boot_flag == 0:
            header_size = 256
            pload_size_pos = 68
            ram_base_pos = 76
            exec_offset_pos = 80

    #get load address and enterpoint.
    for line in ini_info:
        str0 = line.strip()
        item = str0.split()

        if line_no == 0:
            if item[0] == "gnu":
                line_no = 1
                toolchain = 1
                continue
            elif item[0] == "mw":
                line_no = 1
                continue
            else:
                print("invalid ini file!")
                break
        else:
            if toolchain == 1:
                if len(item) < 2:
                    print("Error:")
                    print("\tthe ini file of the input file is invalid!")
                else:
                    if item[1] == ".init":
                        load_addr = item[4]
                    elif item[1] == ".data":
                        if item[3] != item[4]:
                            ram_base = item[4]
                    elif item[1] == ".nvm_header":
                        header_pos = int(item[4], 16)
                        header_size = int(item[2], 16)
                        h_seek = header_pos - int(load_addr, 16)
                        #header_size = int(header_size, 16)
                    elif item[1] == "<_arc_reset>:":
                        enterpoint = item[0]
                    elif item[1] == ".rodata":
                        print("\r\n")
                    else:
                        print("warning: ", item[1])
                        continue
            else:
                if item[2] == ".init_bootstrap,":
                    load_addr = item[4].split("=")[1].split(",")[0]
                    t_base = item[5].split("=")[1]
                elif item[2] == ".rodata,":
                    continue
                elif item[2] == ".nvm_header,":
                    #h_off - t_base --> header in .bin
                    #align with 8bytes
                    h_off = item[5].split("=")[1]
                    #header_pos = int(h_off, 16) - int(t_base, 16)
                    #if header_pos & 0x7 != 0:
                    #    header_pos = (((header_pos >> 3) + 1) << 3)
                    h_seek = int(h_off, 16)
                    #default is 512,
                    #header_size = 512
                elif item[2] == ".data":
                    continue
                else:
                    if len(item) > 6:
                        if item[6] == "_arc_reset":
                            enterpoint = item[1]

    if ram_base == 0:
        ram_base = load_addr
    if boot_flag > 0:
        ram_base = load_addr

    #read nvm header.
    #h_seek = int(header_pos, 16) - int(load_addr, 16)
    #h_seek = header_pos - int(load_addr, 16)
    header_in_file = filename
    if toolchain == 0:
        header_in_file = elfname
    file0 = open(header_in_file, 'rb')
    file0.seek(h_seek)
    header = file0.read(header_size)
    file0.close()

    #fill header, and compute crc32.
    header = bytearray(header)

    bin_size = os.path.getsize(filename)
    a_word = bin_size.to_bytes(length=4, byteorder='little')
    #header[pload_size_pos:pload_size_pos + 3] = a_word
    header[pload_size_pos] = a_word[0]
    header[pload_size_pos + 1] = a_word[1]
    header[pload_size_pos + 2] = a_word[2]
    header[pload_size_pos + 3] = a_word[3]

    ram_base = int(ram_base, 16)
    a_word = ram_base.to_bytes(length=4, byteorder='little')
    #header[ram_base_pos:ram_base_pos + 3] = a_word
    header[ram_base_pos] = a_word[0]
    header[ram_base_pos + 1] = a_word[1]
    header[ram_base_pos + 2] = a_word[2]
    header[ram_base_pos + 3] = a_word[3]

    exec_offset = int(enterpoint, 16) - int(load_addr, 16)
    if (((system_boot_stage == 2) and (xip_flag == 1)) or ((bootloader_method == 1) and (boot_flag > 0))):
        exec_offset = int(enterpoint, 16) - flash_mmap_firmware_base
    a_word = exec_offset.to_bytes(length=4, byteorder='little')
    #header[exec_offset_pos:exec_offset_pos + 3] = a_word
    header[exec_offset_pos] = a_word[0]
    header[exec_offset_pos + 1] = a_word[1]
    header[exec_offset_pos + 2] = a_word[2]
    header[exec_offset_pos + 3] = a_word[3]

    h_crc = crc32_update(header, header_size - 4)
    a_word = h_crc.to_bytes(length=4, byteorder='little')
    #header[header_size - 4:] = a_word
    header[header_size - 4] = a_word[0]
    header[header_size - 3] = a_word[1]
    header[header_size - 2] = a_word[2]
    header[header_size - 1] = a_word[3]

    if (system_boot_stage == 2) or ((system_boot_stage == 3) and (boot_flag > 0)):
        file1 = open("header.bin", 'wb')
        file1.write(header)
        file1.close()
    else:
        file1 = open("tmp.bin", 'wb')
        file1.write(header)
        file1.close()

if __name__ == '__main__' :
    #key = b'\xbb' * 32
    key = b'\xff\xfe\xfd\xfc\xfb\xfa\xf9\xf8\xf7\xf6\xf5\xf4\xf3\xf2\xf1\xf0\xbf\xbe\xbd\xbc\xbb\xba\xb9\xb8\xb7\xb6\xb5\xb4\xb3\xb2\xb1\xb0'
    iv = b'\x00' * 16

    # aes, stage, boot_split, boot_use_hw_crc, boot_use_xip_load_fw, bootloader_method,
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument('-aes', help = "Enable AES encrypt, 1 for AES encrypt enable; 0 for AES encrypt disable [1/0]", default = "0")
    parser.add_argument('-stage', help = "Set boot stage, 2 for 2-stage boot; 3 for 3-stage boot [2/3]", default = "2")
    parser.add_argument('-bsplit', help = "Enable boot split, 1 for split; 0 for non-split [1/0]", default = "0")
    parser.add_argument('-hwcrc', help = "Enable boot use hareware crc, 1 for HWCRC; 0 for SWCRC [1/0]", default = "1")
    parser.add_argument('-flxip', help = "Enable boot use XIP load firmware, 1 for XIP load; 0 for SPI load [1/0]", default = "1")
    parser.add_argument('-bmethod', help = "Bootloader boot method, 1 for XIP; 0 for RAM [1/0]", default = "1")

    # original param: input_bin, output_bin, boot_method, boot_or_firmware
    parser.add_argument('-fbin', help = "input firmware binary file", default = "NULL")
    parser.add_argument('-bbin', help = "input boot binary file", default = "NULL")
    parser.add_argument('-obin', help = "output binary file", default = "NULL")
    parser.add_argument('-fmethod', help = "Firmware XIP boot or RAM boot, 1 for XIP boot; 0 for RAM boot [1/0]", default = "1")
    args = parser.parse_args()

    if args.aes == "1":
        security_flag = 1
    elif args.aes == "0":
        security_flag = 0
    else:
        raise Exception("Invalid aes input")

    if args.stage == "2":
        system_boot_stage = 2
    elif args.stage == "3":
        system_boot_stage = 3
    else:
        raise Exception("Invalid stage input")

    if args.bsplit == "1":
        boot_split = 1
    elif args.bsplit == "0":
        boot_split = 0
    else:
        raise Exception("Invalid boot_split input")

    if args.hwcrc == "1":
        boot_use_hw_crc = 1
    elif args.hwcrc == "0":
        boot_use_hw_crc = 0
    else:
        raise Exception("Invalid boot_use_hwcrc input")

    if args.flxip == "1":
        boot_use_xip_load_fw = 1
    elif args.flxip == "0":
        boot_use_xip_load_fw = 0
    else:
        raise Exception("Invalid boot_use_xip_load_fw input")

    if args.bmethod == "1":
        bootloader_method = 1
    elif args.bmethod == "0":
        bootloader_method = 0
    else:
        raise Exception("Invalid bootloader_method input")

    if args.bbin == "NULL":
        boot_flag = 0
    else:
        boot_flag = 1
        input_bin = args.bbin
    
    if args.fbin == "NULL":
        boot_flag = 1
    else:
        boot_flag = 0
        input_bin = args.fbin
    
    if args.fmethod == "1":
        xip_flag = 1
    elif args.fmethod == "0":
        xip_flag = 0
    else:
        raise Exception("Invalid fmethod input")


    bin_size = os.path.getsize(input_bin)
    if (bin_size % 0x80 != 0):
        w_cnt = 0x80 - (bin_size % 0x80)
        dummy_data = bytes(w_cnt)
        bin_file = open(input_bin, 'ab')
        bin_file.write(dummy_data)
        bin_file.close()

    gen_header(input_bin, boot_flag, xip_flag)


    if security_flag == 0:
        # AES is disabled
        if (system_boot_stage == 2):
            # 2 stage firmware, use sw_crc, and generate firmware.bin (firmware + sw_crc)
            with open(args.obin, 'wb') as f :
                    f.write(combine_crc(gen_1k_stream(word_align(input_bin)), 'data.dat', 'crc.dat'))
                    f.close()

        elif (system_boot_stage == 3):
            if (boot_flag == 0) and (boot_use_hw_crc == 1) :
                # 3 stage firmware with BOOT_USE_HW_CRC=1, use hw_crc, and generate firmware.bin (firmware + hw_crc)
                with open(args.obin, 'wb') as f :
                    firmware = data_append(word_align(input_bin))
                    crc = gen_alps_hw_crc(gen_1k_stream(shuffle_byte_rev_order(data_append(shuffle_word_order(firmware)))))
                    f.write(firmware + crc)
                    f.close() 

            else:
                # 3 stage boot/3 stage firmware with BOOT_USE_HW_CRC=NULL, use sw_crc, and generate boot.bin (boot + sw_crc)
                with open(args.obin, 'wb') as f :
                    f.write(combine_crc(gen_1k_stream(word_align(input_bin)), 'data.dat', 'crc.dat'))
                    f.close()

        # Combine img_header with firmware.bin, and generate firmware_combine.bin
        if (system_boot_stage == 3):
            if (boot_flag == 0):
                # 3 stage firmware, combine image header with firmware
                tmp_name = os.path.basename(args.obin).split('.')[0]
                file1 = open("tmp.bin", 'rb')
                file1.seek(0)
                data = file1.read()
                file1.close()

                file0 = open(args.obin, 'rb')
                file0.seek(0)
                old_data = file0.read()
                file0.close()

                tmp_name = os.path.basename(args.obin).split('.')[0]
                file2 = open(tmp_name + "_combine.bin", 'wb')
                file2.write(data)
                file2.write(old_data)
                file2.close()

            # else:
            #     if boot_split > 0:
            #         tmp_name = os.path.basename(input_bin).split('.')[0]
            #         boot1_raw_file = os.path.abspath(os.path.dirname(input_bin) + "\\" + tmp_name + "_extra.bin")
            #         boot1_file_name = "ota.bin"
            #         with open(boot1_file_name, 'wb') as f :
            #             f.write(combine_crc(gen_1k_stream(word_align(boot1_raw_file)), 'boot1_data.dat', 'boot1_crc.dat'))
            #         f.close()

            #         boot1_file = open(boot1_file_name, 'rb')
            #         boot1_file.seek(0)
            #         data1 = boot1_file.read()
            #         boot1_file.close()

            #         boot_file = open(args.obin, 'ab+')
            #         boot_file.write(data1)
            #         boot_file.close()
    else:
        # AES is enabled
        # Encrypt img_header + crc, calculate firmware

        if xip_flag == 1:
            # XIP mode with AES enable
            if (system_boot_stage == 3) and (boot_flag == 0):
                # 3 stage xip firmware, encrypt image header
                img_header_name = "temp.bin"
                header_file = open("tmp.bin", 'rb')
                header_data = header_file.read(256)
                header_file.close()
                if (bootloader_method == 1):
                    with open(img_header_name, 'wb') as f :
                        if (boot_split == 0) :
                            # encrypt image header under 3 stage boot-no-split xip mode
                            for d  in shuffle_word_order1(encrypt_stream(shuffle_word_order(header_data), key, iv)):
                                f.write(d)
                            f.close()
                else:
                    with open(img_header_name, 'wb') as f :
                        if (boot_split == 0) :
                            # encrypt image header under 3 stage boot-no-split xip mode
                            for d  in shuffle_byte_rev_order1(encrypt_stream(shuffle_byte_rev_order(header_data), key, iv)):
                                f.write(d)
                            f.close()
            if (system_boot_stage == 2):
                # 2 stage xip firmware, use sw_crc
                with open(args.obin, 'wb') as f :
                    for d  in shuffle_word_order1(encrypt_stream(shuffle_word_order(combine_crc(gen_1k_stream(word_align(input_bin)), 'data.dat', 'crc.dat')), key, iv)) :
                        f.write(d)
                    f.close()
            elif (system_boot_stage == 3):
                if (boot_flag == 0) and (boot_use_hw_crc == 1) :
                    # 3 stage xip firmware with BOOT_USE_HW_CRC=1, use hw_crc
                    with open(args.obin, 'wb') as f :
                        firmware = data_append(word_align(input_bin))
                        crc = gen_alps_hw_crc(gen_1k_stream(shuffle_byte_rev_order(data_append(shuffle_word_order(firmware)))))
                        for d  in shuffle_word_order1(encrypt_stream(shuffle_word_order(firmware + crc), key, iv)) :
                            f.write(d)
                        f.close()
                elif ((boot_flag > 0) and (bootloader_method == 1)):
                    # 3 stage xip boot/3 stage firmware with BOOT_USE_HW_CRC=NULL, use sw_crc
                    with open(args.obin, 'wb') as f :
                        for d  in shuffle_word_order1(encrypt_stream(shuffle_word_order(combine_crc(gen_1k_stream(word_align(input_bin)), 'data.dat', 'crc.dat')), key, iv)) :
                            f.write(d)
                        f.close()
                else:
                    # 3 stage xip boot/3 stage firmware with BOOT_USE_HW_CRC=NULL, use sw_crc
                    with open(args.obin, 'wb') as f :
                        for d  in shuffle_word_order1(encrypt_stream(shuffle_word_order(combine_crc(gen_1k_stream(word_align(input_bin)), 'data.dat', 'crc.dat')), key, iv)) :
                            f.write(d)
                        f.close()
        else:
            # RAM mode with AES enable
            if (system_boot_stage == 3) and (boot_flag == 0):
                # 3 stage ram firmware, encrypt image header under 3 stage ram mode
                img_header_name = "tmp.bin"
                header_file = open("tmp.bin", 'rb')
                header_data = header_file.read(256)
                header_file.close()
                if (bootloader_method == 1):
                    with open(img_header_name, 'wb') as f :
                        # encrypt image header under 3 stage boot-no-split ram mode
                        for d  in shuffle_word_order1(encrypt_stream(shuffle_word_order(header_data), key, iv)):
                            f.write(d)
                        f.close()
                else:
                    with open(img_header_name, 'wb') as f :
                        # encrypt image header under 3 stage boot-no-split ram mode
                        for d  in shuffle_byte_rev_order1(encrypt_stream(shuffle_byte_rev_order(header_data), key, iv)):
                            f.write(d)
                        f.close()
            if (system_boot_stage == 2):
                # 2 stage ram firmware
                with open(args.obin, 'wb') as f :
                    for d  in shuffle_byte_rev_order1(encrypt_stream(shuffle_byte_rev_order(combine_crc(gen_1k_stream(word_align(input_bin)), 'data.dat', 'crc.dat')), key, iv)) :
                        f.write(d)
                    f.close()
            elif (system_boot_stage == 3):
                if (boot_use_xip_load_fw == 1):
                    # 3 stage xip firmware with BOOT_USE_HW_CRC=1, use hw_crc
                    if (boot_flag == 0) and (boot_use_hw_crc == 1):
                        with open(args.obin, 'wb') as f :
                            firmware = data_append(word_align(input_bin))
                            crc = gen_alps_hw_crc(gen_1k_stream(shuffle_byte_rev_order(data_append(shuffle_word_order(firmware)))))
                            for d  in shuffle_word_order1(encrypt_stream(shuffle_word_order(firmware + crc), key, iv)) :
                                f.write(d)
                            f.close()
                    elif ((boot_flag > 0) and (bootloader_method == 1)) :
                        # 3 stage xip boot/3 stage firmware with BOOT_USE_HW_CRC=NULL, use sw_crc
                        with open(args.obin, 'wb') as f :
                            for d  in shuffle_word_order1(encrypt_stream(shuffle_word_order(combine_crc(gen_1k_stream(word_align(input_bin)), 'data.dat', 'crc.dat')), key, iv)) :
                                f.write(d)
                            f.close()
                    else:
                        # 3 stage xip boot/3 stage firmware with BOOT_USE_HW_CRC=NULL, use sw_crc
                        with open(args.obin, 'wb') as f :
                            for d  in shuffle_byte_rev_order1(encrypt_stream(shuffle_byte_rev_order(combine_crc(gen_1k_stream(word_align(input_bin)), 'data.dat', 'crc.dat')), key, iv)) :
                                f.write(d)
                            f.close()
                else:
                    # encrypt boot and firmware image under ram mode
                    if (boot_flag == 0) and (boot_use_hw_crc == 1) :
                        # 3 stage firmware with BOOT_USE_HW_CRC=1, use hw_crc
                        with open(args.obin, 'wb') as f :
                            firmware = data_append(word_align(input_bin))
                            crc = gen_alps_hw_crc(gen_1k_stream(shuffle_byte_rev_order(data_append(shuffle_word_order(firmware)))))
                            for d  in shuffle_byte_rev_order1(encrypt_stream(shuffle_byte_rev_order(firmware + crc), key, iv)) :
                                f.write(d)
                            f.close()
                    elif ((boot_flag > 0) and (bootloader_method == 1)):
                        # 3 stage xip boot/3 stage firmware with BOOT_USE_HW_CRC=NULL, use sw_crc
                        with open(args.obin, 'wb') as f :
                            for d  in shuffle_word_order1(encrypt_stream(shuffle_word_order(combine_crc(gen_1k_stream(word_align(input_bin)), 'data.dat', 'crc.dat')), key, iv)) :
                                f.write(d)
                            f.close()
                    else:
                        # 3 stage ram boot/3 stage ram firmware with BOOT_USE_HW_CRC=NULL, use sw_crc
                        with open(args.obin, 'wb') as f :
                            for d  in shuffle_byte_rev_order1(encrypt_stream(shuffle_byte_rev_order(combine_crc(gen_1k_stream(word_align(input_bin)), 'data.dat', 'crc.dat')), key, iv)) :
                                f.write(d)
                            f.close()
        # Combine firmware and image header, and generate firmware_combine.bin
        if (system_boot_stage == 3):
            if (boot_flag == 0):
                tmp_name = os.path.basename(args.obin).split('.')[0]
                file1 = open(img_header_name, 'rb')
                file1.seek(0)
                data = file1.read()
                file1.close()

                file0 = open(args.obin, 'rb')
                file0.seek(0)
                old_data = file0.read()
                file0.close()

                tmp_name = os.path.basename(args.obin).split('.')[0]
                file2 = open(tmp_name + "_combine.bin", 'wb')
                file2.write(data)
                file2.write(old_data)
                file2.close()

            # else:
            #     if boot_split > 0:
            #         tmp_name = os.path.basename(input_bin).split('.')[0]
            #         boot1_raw_file = os.path.abspath(os.path.dirname(input_bin)+"\\"+tmp_name + "_extra.bin")
            #         boot1_file_name = "ota_en.bin"
            #         with open(boot1_file_name, 'wb') as f:
            #             for d  in shuffle_byte_rev_order1(encrypt_stream(shuffle_byte_rev_order(combine_crc(gen_1k_stream(word_align(boot1_raw_file)), 'data.dat', 'crc.dat')), key, iv)):
            #                 f.write(d)
            #             f.close()

            #         boot1_file = open(boot1_file_name, 'rb')
            #         boot1_file.seek(0)
            #         data1 = boot1_file.read()
            #         boot1_file.close()

            #         boot_file = open(args.obin, 'ab+')
            #         boot_file.write(data1)
            #         boot_file.close()
