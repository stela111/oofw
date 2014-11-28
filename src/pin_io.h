#ifndef PIN_IO
#define PIN_IO

#include <cstdint>

/** Interface for modifying single io pins.
 */
class PinIo {
 public:
  /** Pin points out a specific io pin
   */
  struct Pin {
    std::uint8_t pin_no;
  };

  virtual void set(Pin) = 0;
  virtual void clear(Pin) = 0;
  virtual bool get(Pin) = 0;
};

#endif
