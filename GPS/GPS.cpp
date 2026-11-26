#include "fix_quality.hpp"

#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <string_view>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>

namespace {

constexpr char GPS_DEVICE[] = "/dev/ttyACM0";
constexpr int MINIMUM_SATELLITES = 8;
constexpr double MAXIMUM_HDOP = 2.0;
constexpr int MAXIMUM_SENTENCE_LENGTH = 256;
constexpr int MAXIMUM_FIELDS = 20;

struct PositionFix {
    double latitudeDegrees;
    double longitudeDegrees;
    double altitudeMeters;
    double horizontalDilution;  // HDOP: lower values indicate better satellite geometry.
    int qualityCode;
    int satelliteCount;
};

bool isDigit(char character) {
    return character >= '0' && character <= '9';
}

// A field is valid only if the entire piece of text is a number.
bool readWholeNumber(std::string_view text, int &number) {
    if (text.empty() || !isDigit(text.front())) {
        return false;
    }
    const char *end = text.data() + text.size();
    auto result = std::from_chars(text.data(), end, number);
    return result.ec == std::errc{} && result.ptr == end;
}

bool readDecimal(std::string_view text, double &number) {
    if (!text.empty() && text.front() == '+') {
        text.remove_prefix(1);
        if (!text.empty() && text.front() == '-') {
            return false;
        }
    }
    if (text.empty()) {
        return false;
    }
    const char *end = text.data() + text.size();
    auto result = std::from_chars(text.data(), end, number, std::chars_format::fixed);
    return result.ec == std::errc{} && result.ptr == end && std::isfinite(number);
}

bool readCoordinate(std::string_view text, std::string_view hemisphere,
                    bool isLatitude, double &decimalDegrees) {
    // Example: latitude 4807.038,N means 48 degrees plus 7.038 minutes.
    int degreeDigits = isLatitude ? 2 : 3;
    int maximumDegrees = isLatitude ? 90 : 180;
    if (text.size() < static_cast<std::size_t>(degreeDigits + 2)) {
        return false;
    }

    std::string_view minuteText = text.substr(degreeDigits);
    if (!isDigit(minuteText[0]) || !isDigit(minuteText[1])) {
        return false;
    }
    if (minuteText.size() > 2 && (minuteText[2] != '.' || minuteText.size() == 3)) {
        return false;
    }

    int degrees;
    double minutes;
    if (!readWholeNumber(text.substr(0, degreeDigits), degrees) || !readDecimal(minuteText, minutes)) {
        return false;
    }
    if (degrees > maximumDegrees || minutes >= 60 || (degrees == maximumDegrees && minutes != 0)) {
        return false;
    }

    std::string_view positiveDirection = isLatitude ? "N" : "E";
    std::string_view negativeDirection = isLatitude ? "S" : "W";
    if (hemisphere != positiveDirection && hemisphere != negativeDirection) {
        return false;
    }
    decimalDegrees = degrees + minutes / 60.0;
    if (hemisphere == negativeDirection) {
        decimalDegrees = -decimalDegrees;
    }
    return true;
}

// The receiver puts two hexadecimal digits after '*'. Check them before using the data.
std::string_view checkChecksum(std::string_view sentence, std::size_t starPosition) {
    if (starPosition == std::string_view::npos || starPosition + 3 != sentence.size()) {
        return "missing or malformed checksum";
    }
    std::string_view checksumText = sentence.substr(starPosition + 1);
    unsigned int receiverChecksum;
    const char *end = checksumText.data() + checksumText.size();
    auto result = std::from_chars(checksumText.data(), end, receiverChecksum, 16);
    if (result.ec != std::errc{} || result.ptr != end) {
        return "invalid checksum digits";
    }

    unsigned int calculatedChecksum = 0;
    for (char character : sentence.substr(1, starPosition - 1)) {
        calculatedChecksum ^= static_cast<unsigned char>(character);
    }
    if (calculatedChecksum != receiverChecksum) {
        return "checksum mismatch";
    }
    return {};
}

// GGA fields always appear in the same order, even when a field is empty.
std::string_view readPositionFix(std::string_view sentence, PositionFix &fix) {
    std::size_t starPosition = sentence.find('*');
    std::string_view error = checkChecksum(sentence, starPosition);
    if (!error.empty()) {
        return error;
    }

    std::string_view body = sentence.substr(0, starPosition);
    std::string_view fields[MAXIMUM_FIELDS];
    int fieldCount = 0;
    std::size_t fieldStart = 0;
    for (std::size_t index = 0; index <= body.size(); ++index) {
        if (index == body.size() || body[index] == ',') {
            if (fieldCount == MAXIMUM_FIELDS) {
                return "too many fields";
            }
            fields[fieldCount] = body.substr(fieldStart, index - fieldStart);
            ++fieldCount;
            fieldStart = index + 1;
        }
    }

    if (fieldCount < 11 || fields[0].size() != 6 || fields[10] != "M") {
        return "incomplete GGA sentence or unexpected altitude unit";
    }
    if (!readWholeNumber(fields[6], fix.qualityCode)) {
        return "invalid fix quality";
    }
    error = gps::fixQualityError(fix.qualityCode);
    if (!error.empty()) {
        return error;
    }
    if (!readWholeNumber(fields[7], fix.satelliteCount)) {
        return "invalid satellite count";
    }
    if (fix.satelliteCount < MINIMUM_SATELLITES) {
        return "too few satellites";
    }
    if (!readDecimal(fields[8], fix.horizontalDilution)) {
        return "invalid HDOP";
    }
    if (fix.horizontalDilution < 0 || fix.horizontalDilution > MAXIMUM_HDOP) {
        return "HDOP outside configured limit";
    }
    if (!readCoordinate(fields[2], fields[3], true, fix.latitudeDegrees)) {
        return "invalid latitude or hemisphere";
    }
    if (!readCoordinate(fields[4], fields[5], false, fix.longitudeDegrees)) {
        return "invalid longitude or hemisphere";
    }
    if (!readDecimal(fields[9], fix.altitudeMeters)) {
        return "invalid altitude";
    }
    return {};
}

const char *qualityName(int qualityCode) {
    switch (qualityCode) {
        case 1: return "GNSS_FIX";
        case 2: return "DGPS_FIX";
        case 3: return "PPS_FIX";
        case 4: return "RTK_FIXED";
        case 5: return "RTK_FLOAT";
        default: return "OTHER_FIX";
    }
}

void reportRejection(std::string_view reason) {
    using Clock = std::chrono::steady_clock;
    static auto nextWarningTime = Clock::now();
    auto currentTime = Clock::now();
    if (currentTime >= nextWarningTime) {
        std::fprintf(stderr, "[ERROR - DATA REJECTED] %.*s\n",
                     static_cast<int>(reason.size()), reason.data());
        nextWarningTime = currentTime + std::chrono::seconds(5);
    }
}

void processSentence(std::string_view sentence, bool quietMode) {
    if (sentence.size() < 6 || sentence.front() != '$' || sentence.substr(3, 3) != "GGA") {
        return;
    }
    PositionFix fix{};
    std::string_view error = readPositionFix(sentence, fix);
    if (quietMode) {
        return;
    }
    if (!error.empty()) {
        reportRejection(error);
        return;
    }
    std::printf("[VALID - %s] Lat: %.8f, Lon: %.8f, Alt: %.2fm (Sats: %d, HDOP: %.2f)\n",
                qualityName(fix.qualityCode), fix.latitudeDegrees, fix.longitudeDegrees,
                fix.altitudeMeters, fix.satelliteCount, fix.horizontalDilution);
}

// Linux serial settings: 38400 baud, 8 data bits, no parity, one stop bit.
bool configureReceiver(int serialPort, termios &originalSettings) {
    if (tcgetattr(serialPort, &originalSettings) < 0) {
        std::perror("[FATAL] read serial settings");
        return false;
    }
    termios settings = originalSettings;
    cfmakeraw(&settings);
    settings.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    settings.c_cflag |= CLOCAL | CREAD | CS8;
    settings.c_cc[VMIN] = 1;
    settings.c_cc[VTIME] = 0;
    if (cfsetispeed(&settings, B38400) < 0 || cfsetospeed(&settings, B38400) < 0 ||
        tcsetattr(serialPort, TCSANOW, &settings) < 0) {
        std::perror("[FATAL] configure serial port");
        return false;
    }
    return true;
}

volatile sig_atomic_t keepReading = 1;

void requestStop(int) {
    keepReading = 0;
}

bool prepareStopSignals(sigset_t &signalsWhileWaiting) {
    struct sigaction handler{};
    handler.sa_handler = requestStop;
    sigemptyset(&handler.sa_mask);
    if (sigaction(SIGINT, &handler, nullptr) < 0 || sigaction(SIGTERM, &handler, nullptr) < 0) {
        std::perror("[FATAL] install signal handlers");
        return false;
    }

    // Deliver stop signals during ppoll so Ctrl+C also wakes an idle receiver.
    sigset_t stopSignals;
    sigemptyset(&stopSignals);
    sigaddset(&stopSignals, SIGINT);
    sigaddset(&stopSignals, SIGTERM);
    if (sigprocmask(SIG_BLOCK, &stopSignals, &signalsWhileWaiting) < 0) {
        std::perror("[FATAL] set signal mask");
        return false;
    }
    sigdelset(&signalsWhileWaiting, SIGINT);
    sigdelset(&signalsWhileWaiting, SIGTERM);
    return true;
}

bool readReceiver(int serialPort, const sigset_t &signalsWhileWaiting, bool quietMode) {
    char incomingBytes[256];
    std::string sentence;
    sentence.reserve(MAXIMUM_SENTENCE_LENGTH);
    bool sentenceTooLong = false;
    pollfd receiverEvent{serialPort, POLLIN, 0};

    while (keepReading) {
        if (ppoll(&receiverEvent, 1, nullptr, &signalsWhileWaiting) < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::perror("[FATAL] poll serial port");
            return false;
        }
        if (!(receiverEvent.revents & POLLIN)) {
            std::fprintf(stderr, "[FATAL] serial port disconnected or unavailable\n");
            return false;
        }
        ssize_t byteCount = read(serialPort, incomingBytes, sizeof(incomingBytes));
        if (byteCount < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            std::perror("[FATAL] read serial port");
            return false;
        }
        if (byteCount == 0) {
            std::fprintf(stderr, "[FATAL] serial port returned EOF\n");
            return false;
        }

        // A single read can contain part of a sentence or several complete ones.
        for (ssize_t index = 0; index < byteCount; ++index) {
            char character = incomingBytes[index];
            if (character == '\n') {
                if (sentenceTooLong) {
                    if (!quietMode) {
                        reportRejection("sentence too long");
                    }
                } else {
                    if (!sentence.empty() && sentence.back() == '\r') {
                        sentence.pop_back();
                    }
                    processSentence(sentence, quietMode);
                }
                sentence.clear();
                sentenceTooLong = false;
            } else if (sentence.size() < MAXIMUM_SENTENCE_LENGTH) {
                sentence.push_back(character);
            } else {
                sentenceTooLong = true;
            }
        }
    }
    return true;
}

} // namespace

int main(int argc, char **argv) {
    bool quietMode = argc == 2 && std::string_view(argv[1]) == "--quiet";
    if (argc > 1 && !quietMode) {
        std::fprintf(stderr, "Usage: %s [--quiet]\n", argv[0]);
        return 1;
    }

    sigset_t signalsWhileWaiting;
    if (!prepareStopSignals(signalsWhileWaiting)) {
        return 1;
    }
    int serialPort = open(GPS_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serialPort < 0) {
        std::perror("[FATAL] open serial port");
        return 1;
    }
    termios originalSettings{};
    if (!configureReceiver(serialPort, originalSettings)) {
        close(serialPort);
        return 1;
    }

    if (!quietMode) {
        std::printf("[INFO] Reading GPS stream from %s at 38400 baud. Press Ctrl+C to stop.\n", GPS_DEVICE);
    }
    bool finishedNormally = readReceiver(serialPort, signalsWhileWaiting, quietMode);
    if (tcsetattr(serialPort, TCSANOW, &originalSettings) < 0) {
        std::perror("[WARN] Could not restore serial settings");
    }
    if (close(serialPort) < 0) {
        std::perror("[WARN] Could not close serial port");
    }
    return finishedNormally ? 0 : 1;
}
