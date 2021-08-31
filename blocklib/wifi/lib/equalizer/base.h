/*
 * Copyright (C) 2015 Bastian Bloessl <bloessl@ccs-labs.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDED_IEEE802_11_EQUALIZER_BASE_H
#define INCLUDED_IEEE802_11_EQUALIZER_BASE_H

#include <gnuradio/types.hh>
#include <gnuradio/digital/constellation.hh>
#include <string.h>

namespace gr {
namespace wifi {
namespace equalizer {

class base {
protected:
  gr_complex d_H[64];

public:
  virtual ~base(){};
  virtual void equalize(gr_complex *in, int n, gr_complex *symbols,
                        uint8_t *bits,
                        std::shared_ptr<gr::digital::constellation> mod) = 0;
  virtual double get_snr() = 0;
  virtual gr_complex *get_H() { return d_H; }
  virtual void set_H(const gr_complex *H) { memcpy(d_H, H, 64 * sizeof(gr_complex)); }

  static const gr_complex POLARITY[127];
  static const gr_complex LONG[64];
};

} // namespace equalizer
} /* namespace wifigpu */
} /* namespace gr */

#endif /* INCLUDED_IEEE802_11_EQUALIZER_BASE_H */
