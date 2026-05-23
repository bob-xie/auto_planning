#ifndef FOX_VISUAL_MCAP_WRITER_H
#define FOX_VISUAL_MCAP_WRITER_H

#include <string>
#include <vector>
#include <fstream>
#include <cstdint>
#include <ctime>
#include <chrono>
#include <map>

namespace AD {
namespace FoxVisual {

class McapWriter {
public:
    McapWriter();
    ~McapWriter();

    bool open(const std::string& filename);
    void close();

    bool writeChannel(
        const std::string& topic,
        const std::string& message_encoding = "json",
        const std::string& schema_name = "",
        const std::string& schema_encoding = "",
        const std::string& schema_data = "");

    bool writeMessage(
        const std::string& topic,
        const std::string& data,
        uint64_t timestamp = 0);

    bool isOpen() const { return file_.is_open(); }

private:
    std::fstream file_;
    std::map<std::string, uint16_t> topic_to_channel_id_;
    uint16_t next_channel_id_;
    uint64_t start_time_;

    void writeMagic();
    void writeHeader();
    void writeFooter();
    uint64_t getCurrentTime();

    void writeUint8(uint8_t value);
    void writeUint16(uint16_t value);
    void writeUint32(uint32_t value);
    void writeUint64(uint64_t value);
    void writeString(const std::string& value);
    void writeBytes(const std::vector<uint8_t>& value);
};

}  // namespace FoxVisual
}  // namespace AD

#endif  // FOX_VISUAL_MCAP_WRITER_H
