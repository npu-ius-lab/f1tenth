#include <vector>
#include "protocol.h"

void buildCmd(vector<uint8_t> &buf, uint16_t cmd, uint8_t data[], uint8_t data_len)
{
    uint16_t len;
    uint8_t bcc = 0;
    int i;

    buf.push_back(PROTOCOL_HEAD & 0xFF);
    buf.push_back((PROTOCOL_HEAD >> 8) & 0xFF);

    len = 2 + data_len;

    buf.push_back(len & 0xFF);
    buf.push_back((len >> 8) & 0xFF);

    buf.push_back(cmd & 0xFF);
    buf.push_back((cmd >> 8) & 0xFF);

    for (i = 0; i < data_len; i++)
    {
        buf.push_back(data[i]);
    }

    for (i = 4; i < buf.size(); i++)
    {
        bcc ^= buf[i];
    }

    buf.push_back(bcc);
}
