#ifndef WIFI_BOARD_H
#define WIFI_BOARD_H

#include "board.h"

class WifiBoard : public Board {
protected:
    bool wifi_config_mode_ = false;

    WifiBoard();

public:
    virtual std::string GetBoardType() override;
    virtual Http* CreateHttp() override;
    virtual WebSocket* CreateWebSocket() override;
};

#endif // WIFI_BOARD_H
