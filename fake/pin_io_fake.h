#include "pin_io.h"

#include <limits>
#include <stdexcept>

namespace fake {
  
  class PinIo : public ::PinIo {
  public:
    void set(Pin pin) override {
      pins_[pin.pin_no].level = true;
    }
    void clear(Pin pin) override {
      pins_[pin.pin_no].level = false;
    }
    bool get(Pin pin) override {
      return pins_[pin.pin_no].level;
    }

    std::string pin_name(Pin pin) const {
      return pins_[pin.pin_no].name;
    }

    Pin make_pin(std::string name, bool level = false) {
      size_t max = std::numeric_limits<decltype(Pin::pin_no)>::max();
      if (pins_.size() > max) {
	throw(std::runtime_error("fake::PinIo::make_pin out of pins"));
      }
      Pin pin;
      pin.pin_no = static_cast<decltype(pin.pin_no)>(pins_.size());

      PinData pd;
      pd.name = name;
      pd.level = level;
      pins_.push_back(pd);

      return pin;
    }

  private:
    
    struct PinData {
      std::string name;
      bool level;
    };
    
    std::vector<PinData> pins_;
  };
}
