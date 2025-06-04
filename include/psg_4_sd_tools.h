#include <Arduino.h>
#include "SdFat.h"

void make_new_filename(SdFat32 &sd,
                       String &save_filename,
                       const char *filename_base,
                       const char *extension = ".csv") {
    uint16_t file_no = 1;
    save_filename.reserve(64);
    (((save_filename = "") += filename_base) += file_no++) += extension;
    while (sd.exists(save_filename)) {
        (((save_filename = "") += filename_base) += file_no++) += extension;
    }
}

uint32_t list_files(SdFat32 &sd, const char *path = "/", Stream &stream = Serial) {
    uint32_t count = 0;
    File32 root = sd.open(path);
    File32 entry;
    char tmp[256];
    while ((entry = root.openNextFile())) {
        entry.getName(tmp, sizeof(tmp));
        if (entry.isFile()) {
            stream.print("File: ");
            stream.print(tmp);
            stream.print("\tSize = ");
            stream.print(entry.size());
            stream.println(" bytes");
        } else if (entry.isDir()) {
            stream.print("Dir:  ");
            stream.println(tmp);
        }
        entry.close();
        ++count;
    }
    return count;
}

uint32_t list_files(SdFat32 &sd, String &path, Stream &stream = Serial) {
    return list_files(sd, path.c_str(), stream);
}

bool read_file_to_stream(SdFat32 &sd, const char *filename, Stream &stream = Serial) {
    File32 file = sd.open(filename, FILE_READ);
    if (!file) return false;
    while (file.available()) stream.write(file.read());
    file.close();
    return true;
}

bool read_file_to_stream(SdFat32 &sd, String &filename, Stream &stream = Serial) {
    return read_file_to_stream(sd, filename.c_str(), stream);
}

bool delete_path(SdFat32 &sd, const char *path) {
    return sd.remove(path);
}

bool delete_path(SdFat32 &sd, const String &path) {
    return delete_path(sd, path.c_str());
}

void delete_all_in(SdFat32 &sd, const char *path = "/") {
    File32 root = sd.open(path);
    File32 entry;
    char tmp[256];
    while ((entry = root.openNextFile())) {
        entry.getName(tmp, sizeof(tmp));
        delete_path(sd, tmp);
    }
}

void delete_all_in(SdFat32 &sd, const String &path) {
    delete_all_in(sd, path.c_str());
}

void open_for_read(SdFat32 &sd, File32 &file, const char *filename) {
    file = sd.open(filename, FILE_READ);
}

void open_for_read(SdFat32 &sd, File32 &file, const String &filename) {
    open_for_read(sd, file, filename.c_str());
}

void open_for_write(SdFat32 &sd, File32 &file, const char *filename) {
    file = sd.open(filename, FILE_WRITE);
}

void open_for_write(SdFat32 &sd, File32 &file, const String &filename) {
    open_for_write(sd, file, filename.c_str());
}

void open_for_append(SdFat32 &sd, File32 &file, const char *filename) {
    open_for_write(sd, file, filename);
    file.seekEnd();
}

void open_for_append(SdFat32 &sd, File32 &file, const String &filename) {
    open_for_append(sd, file, filename.c_str());
}
