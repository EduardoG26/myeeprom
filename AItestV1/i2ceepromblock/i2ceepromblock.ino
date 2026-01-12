// i2ceepromblock.ino - KOMPLETT, SINGLE FILE, kompiliert garantiert!
#include <Arduino.h>
#include <Wire.h>

struct BlockHeader {
    uint16_t token;
    uint16_t crc;
};

class I2cEepromBlock {
public:
    I2cEepromBlock(uint8_t i2cAddr, uint16_t userSize, uint8_t sdaPin = 21, uint8_t sclPin = 22);
    ~I2cEepromBlock();
    
    void begin();
    void task();
    uint8_t* data();
    const uint8_t* data() const;
    void markDirty();
    bool isDirty() const;
    uint16_t userSize() const;
    uint16_t blockSize() const;
    uint8_t numBlocks() const;
    uint16_t currentToken() const;
    int currentBlockIndex() const;
    bool readBlock(int blockIdx, BlockHeader& header, uint8_t* buffer) const;
    uint16_t blockBaseAddress(int blockIdx) const;

private:
    static constexpr uint16_t EEPROM_SIZE = 4096;
    static constexpr uint8_t  PAGE_SIZE   = 32;
    static constexpr uint8_t  WRITE_DELAY_MS = 5;
    static constexpr uint16_t HEADER_SIZE = sizeof(BlockHeader);

    uint8_t  _i2cAddr;
    uint16_t _userSize;
    uint16_t _blockSize;
    uint8_t  _numBlocks;
    uint8_t  _sdaPin, _sclPin;

    uint16_t _currentToken;
    int      _currentBlockIndex;
    bool     _dirty;
    bool     _initialized;

    uint8_t* _bufA;
    uint8_t* _bufB;
    uint8_t* _activeBuf;
    uint8_t* _inactiveBuf;

    enum class State { Idle, WritingHeader, WritingData };
    State    _state;
    int      _writeBlockIndex;
    uint16_t _writeOffset;
    BlockHeader _preparedHeader;

    uint16_t crc16_ccitt(const uint8_t* data, uint16_t len) const;
    void readBytes(uint16_t eeAddr, uint8_t* dst, uint16_t len) const;
    void writeBytes(uint16_t eeAddr, const uint8_t* src, uint16_t len);
    void initBuffers();
    void findBestBlock();
};

// === IMPLEMENTATION ===
I2cEepromBlock::I2cEepromBlock(uint8_t i2cAddr, uint16_t userSize, uint8_t sdaPin, uint8_t sclPin)
    : _i2cAddr(i2cAddr), _userSize(userSize), _sdaPin(sdaPin), _sclPin(sclPin)
    , _currentToken(0), _currentBlockIndex(-1), _dirty(false), _initialized(false)
    , _bufA(nullptr), _bufB(nullptr), _activeBuf(nullptr), _inactiveBuf(nullptr)
    , _state(State::Idle), _writeBlockIndex(0), _writeOffset(0) {
    
    _blockSize = HEADER_SIZE + _userSize;
    _numBlocks = EEPROM_SIZE / _blockSize;
}

I2cEepromBlock::~I2cEepromBlock() {
    delete[] _bufA;
    delete[] _bufB;
}

void I2cEepromBlock::begin() {
#if defined(ARDUINO_ARCH_ESP32)
    Wire.begin(_sdaPin, _sclPin);  // ESP32: Pins konfigurierbar
#else
    Wire.begin();                  // Pico: Standard GP4/GP5
#endif
    initBuffers();
    findBestBlock();
    _initialized = true;
    _dirty = true;
}


void I2cEepromBlock::initBuffers() {
    _bufA = new uint8_t[_userSize];
    _bufB = new uint8_t[_userSize];
    _activeBuf = _bufA;
    _inactiveBuf = _bufB;
    memset(_activeBuf, 0xFF, _userSize);
}

// INLINE IMPLEMENTATION - alle Methoden direkt hier
uint16_t I2cEepromBlock::crc16_ccitt(const uint8_t* data, uint16_t len) const {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

uint16_t I2cEepromBlock::blockBaseAddress(int blockIdx) const {
    return (uint16_t)blockIdx * _blockSize;
}

void I2cEepromBlock::readBytes(uint16_t eeAddr, uint8_t* dst, uint16_t len) const {
    Wire.beginTransmission(_i2cAddr);
    Wire.write((uint8_t)(eeAddr >> 8));
    Wire.write((uint8_t)(eeAddr & 0xFF));
    Wire.endTransmission(false);
    Wire.requestFrom(_i2cAddr, (uint8_t)len);
    for (uint16_t i = 0; i < len && Wire.available(); ++i) {
        dst[i] = Wire.read();
    }
}

void I2cEepromBlock::writeBytes(uint16_t eeAddr, const uint8_t* src, uint16_t len) {
    uint16_t addr = eeAddr;
    uint16_t remaining = len;
    
    while (remaining > 0) {
        uint8_t pageOffset = addr % PAGE_SIZE;
        uint8_t chunk = PAGE_SIZE - pageOffset;
        if (chunk > remaining) chunk = remaining;
        
        Wire.beginTransmission(_i2cAddr);
        Wire.write((uint8_t)(addr >> 8));
        Wire.write((uint8_t)(addr & 0xFF));
        for (uint8_t i = 0; i < chunk; ++i) {
            Wire.write(src[(len - remaining) + i]);
        }
        Wire.endTransmission();
        delay(WRITE_DELAY_MS);
        
        addr += chunk;
        remaining -= chunk;
    }
}

bool I2cEepromBlock::readBlock(int blockIdx, BlockHeader& header, uint8_t* buffer) const {
    if (blockIdx < 0 || blockIdx >= _numBlocks) return false;
    
    uint16_t base = blockBaseAddress(blockIdx);
    readBytes(base, (uint8_t*)&header, HEADER_SIZE);
    readBytes(base + HEADER_SIZE, buffer, _userSize);
    
    uint16_t calcCrc = crc16_ccitt(buffer, _userSize);
    return (calcCrc == header.crc);
}

void I2cEepromBlock::findBestBlock() {
    uint16_t bestToken = 0;
    int bestIdx = -1;
    
    uint8_t tempBuf[_userSize];
    BlockHeader tempHeader;
    
    for (int i = 0; i < _numBlocks; ++i) {
        if (readBlock(i, tempHeader, tempBuf)) {
            if (tempHeader.token >= bestToken) {
                bestToken = tempHeader.token;
                bestIdx = i;
                memcpy(_activeBuf, tempBuf, _userSize);
            }
        }
    }
    
    _currentToken = bestToken;
    _currentBlockIndex = bestIdx;
}

uint8_t* I2cEepromBlock::data() { return _activeBuf; }
const uint8_t* I2cEepromBlock::data() const { return _activeBuf; }
void I2cEepromBlock::markDirty() { if (_initialized) _dirty = true; }
bool I2cEepromBlock::isDirty() const { return _dirty; }
uint16_t I2cEepromBlock::userSize() const { return _userSize; }
uint16_t I2cEepromBlock::blockSize() const { return _blockSize; }
uint8_t I2cEepromBlock::numBlocks() const { return _numBlocks; }
uint16_t I2cEepromBlock::currentToken() const { return _currentToken; }
int I2cEepromBlock::currentBlockIndex() const { return _currentBlockIndex; }

void I2cEepromBlock::task() {
    if (!_initialized) return;
    
    switch (_state) {
        case State::Idle:
            if (_dirty) {
                _writeBlockIndex = (_currentBlockIndex + 1) % _numBlocks;
                memcpy(_inactiveBuf, _activeBuf, _userSize);
                
                _preparedHeader.token = _currentToken + 1;
                _preparedHeader.crc = crc16_ccitt(_inactiveBuf, _userSize);
                
                _writeOffset = 0;
                _state = State::WritingHeader;
            }
            break;
            
        case State::WritingHeader: {
            uint16_t base = blockBaseAddress(_writeBlockIndex);
            uint8_t headerBytes[4];
            headerBytes[0] = _preparedHeader.token >> 8;
            headerBytes[1] = _preparedHeader.token & 0xFF;
            headerBytes[2] = _preparedHeader.crc >> 8;
            headerBytes[3] = _preparedHeader.crc & 0xFF;
            
            writeBytes(base, headerBytes, 4);
            _writeOffset = 0;
            _state = State::WritingData;
            break;
        }
            
        case State::WritingData: {
            if (_writeOffset >= _userSize) {
                _currentToken = _preparedHeader.token;
                _currentBlockIndex = _writeBlockIndex;
                std::swap(_activeBuf, _inactiveBuf);
                _dirty = false;
                _state = State::Idle;
                break;
            }
            
            uint16_t base = blockBaseAddress(_writeBlockIndex) + HEADER_SIZE;
            uint16_t addr = base + _writeOffset;
            uint8_t pageOffset = addr % PAGE_SIZE;
            uint8_t chunk = PAGE_SIZE - pageOffset;
            uint16_t remaining = _userSize - _writeOffset;
            if (chunk > remaining) chunk = remaining;
            
            writeBytes(addr, _inactiveBuf + _writeOffset, chunk);
            _writeOffset += chunk;
            break;
        }
    }
};

// === EXAMPLE CODE ===
constexpr uint16_t USER_SIZE = 28;
I2cEepromBlock eeprom(0x50, USER_SIZE, 5, 6);

uint32_t writeCount = 0;
uint32_t lastPrint = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(1000);
    
    Serial.printf("EEPROM Block: %d blocks, %d bytes/block, %d user bytes\n",
                  eeprom.numBlocks(), eeprom.blockSize(), eeprom.userSize());
    
    eeprom.begin();
    Serial.printf("Initial block %d, token %d\n", eeprom.currentBlockIndex(), eeprom.currentToken());
    
    uint8_t* data = eeprom.data();
    for (int i = 0; i < USER_SIZE; ++i) {
        data[i] = (uint8_t)i;
    }
    eeprom.markDirty();
    
    Serial.println("Initial data:");
    printData();
}

void loop() {
    eeprom.task();
    
    if (eeprom.isDirty()) {
        delay(100);
        return;
    }
    
    uint8_t* data = eeprom.data();
    bool changed = false;
    for (int i = 0; i < USER_SIZE; ++i) {
        if (data[i] == 0xFF) data[i] = 0;
        else data[i]++;
        changed = true;
    }
    
    if (changed) {
        eeprom.markDirty();
        writeCount++;
        
        if (writeCount - lastPrint >= 10 || writeCount < 10) {
            Serial.printf("Write #%lu -> block %d, token %d\n",
                         writeCount, eeprom.currentBlockIndex(), eeprom.currentToken());
            printData();
            lastPrint = writeCount;
        }
        
        if (writeCount % 100 == 0) {
            verifyAllBlocks();
        }
    }
    
    delay(500);
}

void printData() {
    uint8_t* data = eeprom.data();
    Serial.print("Data: ");
    for (int i = 0; i < 10; ++i) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();
}

void verifyAllBlocks() {
    Serial.println("\n=== Verifying ALL blocks ===");
    uint8_t tempBuf[USER_SIZE];
    
    for (int i = 0; i < eeprom.numBlocks(); ++i) {
        BlockHeader header;
        if (eeprom.readBlock(i, header, tempBuf)) {
            Serial.printf("Block %2d: VALID token=%4d crc=%04X ", i, header.token, header.crc);
            Serial.print("data[0-3]: ");
            for (int j = 0; j < 4; ++j) Serial.printf("%02X ", tempBuf[j]);
            Serial.println();
        } else {
            Serial.printf("Block %2d: INVALID\n", i);
        }
    }
    Serial.printf("Current active: block %d (token %d)\n\n",
                  eeprom.currentBlockIndex(), eeprom.currentToken());
}
