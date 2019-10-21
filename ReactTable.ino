//#define FASTLED_ESP32_I2S
#include <FastLED.h>
#include <adel.h>

// === Pattern mode =========================================================

/** Mode
 *  This code supports four different animations. You can choose one of them, 
 *  or set "cycle" to true to cycle through all of them at some set interval.
 */
enum Mode { SolidMode, ConfettiMode, SpinnerMode, GearMode, FireMode,
            SurfaceMode, RippleMode, DiffusionMode, AttractorMode, MirrorMode };

#define NUM_MODES 10

// -- Mode choice
Mode g_Mode = SurfaceMode;

// -- To cycle modes, set cycle to true and choose an interval (in milliseconds)
bool g_Cycle = false;
const int TIME_PER_PATTERN = 15000;

#define MODE_PIN 14

// === One-cell pin settings ================================================

/** One cell mode
 *  
 *  If you only have one cell, then you only need two pins: one pin to read 
 *  the IR analog input and one pin to drive the LED ring. You can ignore the
 *  rest of the configuration.
 */

 #define ONE_CELL_MODE false

 #define IR_INPUT_PIN 27
 #define LED_PIN 26

 // === Multi-cell pin settings ==============================================

/** IR channel selector
 *
 * Each analog multiplexer takes 16 analog inputs and produces one The
 * particular output is chosen by setting four digital inputs to the
 * binary representation of the analog input desired. For example, to
 * read input number 13, which is 1101 in binary, set bit 3 to HIGH,
 * bit 2 to HIGH bit 1 to LOW, and bit 0 to HIGH.
 *
 * The four definitions below indicate which pins on the
 * microcontroller are connected to the four digital inputs on the
 * multiplexer.
 */
#define IR_CHANNEL_BIT_0 5
#define IR_CHANNEL_BIT_1 18
#define IR_CHANNEL_BIT_2 23
#define IR_CHANNEL_BIT_3 19

/** IR input pins
 *
 * My design requires four multiplexers because we have 61 IR analog
 * inputs.  The way I set it up, four digital inputs are connected to
 * all four of the multiplexers. Each multiplexer then has its own
 * input pin to read the value.  To read any one of the 64 possible
 * analog inputs, we take the low 4 bits of the input number and send
 * that to the channel selectors (above). Then we read in the input on
 * the pin specified by the next two bits of the input.
 */
int IR_INPUTS[] = {32, 33, 34, 35}; // { 27, 33, 34, 35 };

/** Cell configuration
 *
 * My table has 61 cells, with 12 WS2812 LEDs per cell.
 */
#define NUM_CELLS 61
#define LEDS_PER_CELL 12
#define NUM_LEDS (NUM_CELLS * LEDS_PER_CELL)

class Cell;
Cell * g_Cells[NUM_CELLS];

/** Storage for LEDs */
CRGB g_LEDs[NUM_LEDS];

/** Kind of LEDs */
#define COLOR_ORDER GRB
#define CHIPSET     WS2812

/** LED strip setup
 *  Set up the wiring of the LED strips here. I've wired mine so that each pair of 
 *  columns of rings (a total of 11 rings) has a data input. That's 11 * 12 = 132
 *  LEDs on each pin, except for the last one.
 */
void LED_setup()
{
    FastLED.addLeds<CHIPSET, 17, COLOR_ORDER>(g_LEDs, 132*0, 132).setCorrection( TypicalLEDStrip );
    FastLED.addLeds<CHIPSET, 16, COLOR_ORDER>(g_LEDs, 132*1, 132).setCorrection( TypicalLEDStrip );
    FastLED.addLeds<CHIPSET,  4, COLOR_ORDER>(g_LEDs, 132*2, 132).setCorrection( TypicalLEDStrip );
    FastLED.addLeds<CHIPSET,  2, COLOR_ORDER>(g_LEDs, 132*3, 132).setCorrection( TypicalLEDStrip );
    FastLED.addLeds<CHIPSET, 15, COLOR_ORDER>(g_LEDs, 132*4, 132).setCorrection( TypicalLEDStrip );
    FastLED.addLeds<CHIPSET, 12, COLOR_ORDER>(g_LEDs, 132*5, 72).setCorrection( TypicalLEDStrip );
}

/** Animation speed */
#define FRAMES_PER_SECOND 30

/** Default brightness */
uint8_t g_Brightness = 40;

/** Cell mapping
 *
 * This data structure holds the physical information about a cell:
 *     ir_index: the index of the IR sensor in the multiplexer
 *     ring_index: the index of the LED ring in the logical "strip" of LEDs
 *     x,y: the position of the center of the cell in centimeters
*/

struct CellMapEntry
{
    uint8_t    m_ir_index;
    uint16_t   m_ring_index;
    uint8_t    m_x;
    uint8_t    m_y;
};

CellMapEntry g_CellMap[] = {
    {  0, 60, 2,  2 }, {  1, 49, 12,  2 }, {  2, 38, 22,  2 }, {  3, 27, 32,  2 }, {  4, 16, 42,  2 }, {  5, 5, 52,  2 },
    {  6, 54, 7,  5 }, {  7, 43, 17,  5 }, {  8, 32, 27,  5 }, {  9, 21, 37,  5 }, { 10, 10, 47,  5 },
    { 11, 59, 2,  8 }, { 12, 48, 12,  8 }, { 13, 37, 22,  8 }, { 14, 26, 32,  8 }, { 15, 15, 42,  8 }, { 16, 4, 52,  8 },
    { 17, 53, 7, 11 }, { 18, 42, 17, 11 }, { 19, 31, 27, 11 }, { 20, 20, 37, 11 }, { 21,  9, 47, 11 },
    { 22, 58, 2, 14 }, { 23, 47, 12, 14 }, { 24, 36, 22, 14 }, { 25, 25, 32, 14 }, { 26, 14, 42, 14 }, { 27, 3, 52, 14 },
    { 28, 52, 7, 17 }, { 29, 41, 17, 17 }, { 30, 30, 27, 17 }, { 31, 19, 37, 17 }, { 32,  8, 47, 17 },
    { 33, 57, 2, 20 }, { 34, 46, 12, 20 }, { 35, 35, 22, 20 }, { 36, 24, 32, 20 }, { 37, 13, 42, 20 }, { 38, 2, 52, 20 },
    { 39, 51, 7, 23 }, { 40, 40, 17, 23 }, { 41, 29, 27, 23 }, { 42, 18, 37, 23 }, { 43,  7, 47, 23 },
    { 44, 56, 2, 26 }, { 45, 45, 12, 26 }, { 46, 34, 22, 26 }, { 47, 23, 32, 26 }, { 48, 12, 42, 26 }, { 49, 1, 52, 26 },
    { 50, 50, 7, 29 }, { 51, 39, 17, 29 }, { 52, 28, 27, 29 }, { 53, 17, 37, 29 }, { 54,  6, 47, 29 },
    { 55, 55, 2, 32 }, { 56, 44, 12, 32 }, { 57, 33, 22, 32 }, { 58, 22, 32, 32 }, { 59, 11, 42, 32 }, { 60, 0, 52, 32 }
};

/** Surface coordinate system
 *
 * The surface is a logical 2-D grid measured in centimeters. In surface 
 * mode, the LEDs sample their values from this single grid, allowing the 
 * entire table to be treated as a single image.
 */

#define SURFACE_WIDTH  54
#define SURFACE_HEIGHT 34

/** Surface
 * 
 * Fill this 2-D grid with values to display a single image.
 */
uint8_t  g_Surface[SURFACE_WIDTH][SURFACE_HEIGHT];

/** Ring coordinate mapping
 *
 * Precompute the position of each LED relative to the surface coordinate
 * system. The result is a sampling pattern for each LED: surface elements
 * and ratios of mixing those elements.
 * 
 * My model: a ring has a diameter of 4cm; each LED is .5cm square, so the 
 * inner diameter of the ring is 3cm. The middle of the ring is at 3.5cm.
 */

struct SurfaceSample
{
    int dx;
    int dy;
    int top_left_part;
    int top_right_part;
    int bottom_left_part;
    int bottom_right_part;
};

SurfaceSample g_SurfaceSamples[LEDS_PER_CELL];

int modi(float v, int * i)
{
    double v_whole, v_frac;
    v_frac = modf(v, &v_whole);
    int vi = (int) v_whole;
    int vf = (int) (10.0 * v_frac);

    *i = vi;
    return vf;
}

void computePixelOffsets()
{
    Serial.println();
    for (int i = 0; i < LEDS_PER_CELL; i++) {
        // -- Divide the ring into 12 equal angles
        float frac = ((float) i) / ((float) LEDS_PER_CELL);
        float angle = frac * 3.1415926535897 * 2.0;
        
        // -- X and Y along a ring of diameter 3.0cm
        //    These values correspond to the top left corner of each LED
        float x = (cos(angle) + 1.0) * 1.5;
        float y = (sin(angle) + 1.0) * 1.5;

        // -- Figure out how much of each surface pixel should be sampled by the LED
        int xl, xlf;
        xlf = modi(x, &xl);

        int xr, xrf;
        xrf = modi(x + 0.99, &xr);

        int yt, ytf;
        ytf = modi(y, &yt);

        int yb, ybf;
        ybf = modi(y + 0.99, &yb);

        int top_left = (10 - xlf) * (10 - ytf);
        
        int top_right = 0;
        if (xl != xr) {
            top_right = xrf * (10 - ytf);
        }
        
        int bottom_left = 0;
        if (yt != yb) {
            bottom_left = (10 - xlf) * ybf;
        }
        
        int bottom_right = 0;
        if ((xl != xr) && (yt != yb)) {
            bottom_right = xrf * ybf;
        }

        g_SurfaceSamples[i].dx = xl;
        g_SurfaceSamples[i].dy = yt;
        g_SurfaceSamples[i].top_left_part = top_left;
        g_SurfaceSamples[i].top_right_part = top_right;
        g_SurfaceSamples[i].bottom_left_part = bottom_left;
        g_SurfaceSamples[i].bottom_right_part = bottom_right;
        
        Serial.print("Pixel "); Serial.print(i); Serial.print(" at ");
        Serial.print(x); Serial.print(" , "); Serial.print(y);
        Serial.print(" : \n");
        Serial.print("    X left  : "); Serial.print(xl); Serial.print("+"); Serial.print(xlf); Serial.println();
        Serial.print("    X right : "); Serial.print(xr); Serial.print("+"); Serial.print(xrf); Serial.println();
        Serial.print("    Y top   : "); Serial.print(yt); Serial.print("+"); Serial.print(ytf); Serial.println();
        Serial.print("    Y bottom: "); Serial.print(yb); Serial.print("+"); Serial.print(ybf); Serial.println();
        Serial.print("    Top left     : "); Serial.print(top_left);
        Serial.print("    Top right    : "); Serial.print(top_right);
        Serial.print("    Bottom left  : "); Serial.print(bottom_left);
        Serial.print("    Bottom right : "); Serial.print(bottom_right); Serial.println();
    }        
}

// === Cells ================================================================

/** Cell 
 * 
 *  Each cell is managed by an instance of this class, which includes
 *  all the data and methods needed to read the IR input and set the
 *  corresponding ring of LEDs.
 *
 */
class Cell
{
protected:
    // -- LED ring information
    uint16_t        m_led_index;
    CRGBPalette16   m_palette;
    
    // -- IR sensor
    int             m_ir_input;
    int             m_ir_channel;
    uint16_t        m_ir_min;
    uint16_t        m_ir_max;
    uint8_t         m_ir_channel_selector[4];
    uint16_t        m_level;

    // -- Physical position
    int             m_center_x;
    int             m_center_y;
    int             m_left;
    int             m_top;

    // -- Data for the patterns
    uint16_t        m_position;
    byte            m_heat[LEDS_PER_CELL];
    bool            m_new_pattern;

    struct Flame {
        uint8_t  fuel;
        uint8_t  heat;
    };

    Flame           m_flames[LEDS_PER_CELL];

public:
    Cell( CellMapEntry& info )
        : m_led_index(info.m_ring_index * LEDS_PER_CELL),
          m_palette(RainbowColors_p),
          m_ir_input(info.m_ir_index >> 4),
          m_ir_channel(info.m_ir_index & 0xF),
          m_ir_min(100),
          m_ir_max(0),
          m_level(0),
          m_center_x(info.m_x),
          m_center_y(info.m_y),
          m_position(0),
          m_new_pattern(true)
    {
        // -- Precompute the IR selector signal
        m_ir_channel_selector[0] = (m_ir_channel & 0x1) ? HIGH : LOW;
        m_ir_channel_selector[1] = (m_ir_channel & 0x2) ? HIGH : LOW;
        m_ir_channel_selector[2] = (m_ir_channel & 0x4) ? HIGH : LOW;
        m_ir_channel_selector[3] = (m_ir_channel & 0x8) ? HIGH : LOW;

        // -- Compute top left for convenience
        m_left = m_center_x - 2;
        m_top  = m_center_y - 2;
    }

    // ----- Getters --------

    int getCenterX() const { return m_center_x; }
    int getCenterY() const { return m_center_y; }

    // ----- IR Sensors -----

    /** Set the IR max and min
     *  Determined by the calibration phase.
     */
    void setIRMax(uint16_t ir_max) {
        m_ir_max = ir_max;
    }

    void setIRMin(uint16_t ir_min) {
        m_ir_min = ir_min;
    }

    /** Read raw IR value
     *  We can have several MUXs, each with 16 channels. To read a specific
     *  IR value, we specify which MUX (the "input") and which channel.
     */
    uint16_t rawIR()
    {
        uint16_t val;
        
        if (ONE_CELL_MODE) {
            val = analogRead(IR_INPUT_PIN);
        } else {
            // -- Select the channel
            digitalWrite(IR_CHANNEL_BIT_0, m_ir_channel_selector[0]);
            digitalWrite(IR_CHANNEL_BIT_1, m_ir_channel_selector[1]);
            digitalWrite(IR_CHANNEL_BIT_2, m_ir_channel_selector[2]);
            digitalWrite(IR_CHANNEL_BIT_3, m_ir_channel_selector[3]);

            // -- Finally, read the analog value
            val = analogRead(IR_INPUTS[m_ir_input]);
        }
        
        return val;
    }

    /** Sense IR
     *  Read and map to the calibrated range
     */
    uint8_t senseIR()
    {
        uint16_t val = rawIR();
        
        // -- Pin the value in between the min and max from calibration
        if (val < m_ir_min) val = m_ir_min;
        if (val > m_ir_max) val = m_ir_max;
        
        // -- Map to 8-bit value
        uint8_t level = map(val, m_ir_min, m_ir_max, 0, 255);
        return level;
    }

    /** Sense IR with decay 
     *
     *  This version decays the IR value slowly, causing the visual
     *  effects to linger.
     */
    uint8_t senseIRwithDecay(uint8_t down_speed, uint8_t up_speed)
    {
        uint8_t cur_level = senseIR();
        
        if (cur_level < m_level) {
            uint8_t new_level = qsub8(m_level, down_speed);
            if (cur_level < new_level) {
                m_level = new_level;
            } else {
                m_level = cur_level;
            }
        } else {
            if (cur_level > m_level) {
                uint8_t new_level = qadd8(m_level, up_speed);
                if (cur_level > new_level) {
                    m_level = new_level;
                } else {
                    m_level = cur_level;
                }
            }
        }
        return m_level;
    }

    // ----- Set LEDs in this ring -----

    void setLED(int local_index, CRGB color) {
        if (local_index < 0) local_index = 0;
        if (local_index >= LEDS_PER_CELL) local_index = LEDS_PER_CELL - 1;
        g_LEDs[m_led_index + local_index] = color;
    }

    void setLEDHue(int local_index, uint8_t hue, uint8_t brightness = 255) {
        // -- Get the color from the palette
        CRGB color = ColorFromPalette(m_palette, hue);
        if (brightness < 255) color.nscale8_video(brightness);
        setLED(local_index, color);
    }

    void setAllLEDs(CRGB color) {
        for (int i = 0; i < LEDS_PER_CELL; i++) {
            g_LEDs[m_led_index + i] = color;
        }
    }

    void setAllLEDsHue(uint8_t hue, uint8_t brightness = 255) {
        // -- Get the color from the palette
        CRGB color = ColorFromPalette(m_palette, hue);
        if (brightness < 255) color.nscale8_video(brightness);
        setAllLEDs(color);
    }

    void fadeToBlackBy(uint8_t howmuch) {
        for (int i = 0; i < LEDS_PER_CELL; i++) {
            g_LEDs[m_led_index + i].fadeToBlackBy(howmuch);
        }
    }

    // ----- Dithered pixels -----

    /* Set a pixel
     *
     *  In these methods, the position is expressed as a 16-bit
     *  fixed-precision number that maps to whatever number of actual
     *  LEDs there are in the strip/ring. This function dithers the
     *  adjacent pixels to create a smooth transition.  It needs to be
     *  fast!
     */
    void setPixel(uint16_t pos, CRGB color) 
    {
        // -- Scale the position from 0-0xFFFF to 0-m_num_leds
        //    but keep the fractional part. The result is a 16-bit
        //    value where the high 8 bits are in [0,num_leds] and
        //    the low 8 bits are the faction in [0,0xFF]
        uint32_t pos32 = (uint32_t) pos;
        uint16_t scaled_pos = (pos32 * LEDS_PER_CELL) >> 8;
        
        // -- Break the position into the whole and fractional parts. The
        //    fractional part is represented by the brightness of two
        //    adjacent LEDs.
        int indexA = scaled_pos >> 8;
        uint8_t offsetA = scaled_pos & 0xFF;

        // -- Pixels can straddle two LEDs, so compute the other partial LED.
        int indexB;
        if (indexA == 0) indexB = LEDS_PER_CELL - 1;
        else indexB = indexA - 1;
        uint8_t offsetB = 0xFF - offsetA; // A little less than full on

        // -- Scale the brightness and assign the LEDs
        if (offsetA > 10) nblend(g_LEDs[m_led_index + indexA], color, offsetA);
        if (offsetB > 10) nblend(g_LEDs[m_led_index + indexB], color, offsetB);
    }

    /** Set a pixel
     * This version just chooses the color from the palette.
     */
    void setPixelHue(uint16_t pos, uint8_t hue, uint8_t brightness = 255) 
    {
        // -- Get the color from the palette
        CRGB color = ColorFromPalette(m_palette, hue);
        if (brightness < 255) color.nscale8_video(brightness);
        setPixel(pos, color);
    }
    
    // ----- Patterns -----

    // -- Call this method to trigger pattern initialization
    void newPattern() { m_new_pattern = true; }

    void SolidPattern()
    {
        if (m_new_pattern) {
            m_palette = RainbowColors_p;
            m_new_pattern = false;
        }
        uint8_t level = senseIRwithDecay(12, 4);
        if (level > 230) level = 230;
        setAllLEDsHue(level);
    }
    
    void ConfettiPattern()
    {
        if (m_new_pattern) {
            m_palette = RainbowColors_p;
            m_new_pattern = false;
        }
        fadeToBlackBy(80);
        uint8_t level = senseIRwithDecay(15, 4);
        uint8_t prob = 255 - level;
        uint8_t coin = random8();
        if (coin < prob) {
            int pos = random8(LEDS_PER_CELL);
            setLEDHue(pos, level + random8(64));
        }
    }

    void SpinnerPattern(int min_speed, int max_speed)
    {
        // -- Initialize if necessary
        if (m_new_pattern) {
            m_position = random16();
            m_palette = RainbowColors_p;
            m_new_pattern = false;
        }

        setAllLEDs(CRGB::Black); // fadeToBlackBy(80);
        uint8_t level = senseIRwithDecay(12, 4);
        int speed = map(level, 0, 255, max_speed, min_speed);
        m_position += speed;
        if (level > 230) level = 230;
        setPixelHue(m_position, level);
    }

    void GearPattern(int min_speed, int max_speed)
    {
        // -- Initialize if necessary
        if (m_new_pattern) {
            m_position = random16();
            m_palette = RainbowColors_p;
            m_new_pattern = false;
        }

        setAllLEDs(CRGB::Black); // fadeToBlackBy(80);
        uint8_t level = senseIRwithDecay(12, 4);
        int speed = map(level, 0, 255, max_speed, min_speed);
        m_position += speed;
        if (level > 230) level = 230;
        setPixelHue(m_position, level);
        setPixelHue(m_position + 0x5555, level);
        setPixelHue(m_position + 0xAAAA, level);
    }

    void FirePattern(int sparking, int burn_rate)
    {
        if (m_new_pattern) {
            m_palette = HeatColors_p;
            for (int i = 0; i < LEDS_PER_CELL; i++) {
                m_flames[i].heat = 0;
                m_flames[i].fuel = random(40);
            }
            m_new_pattern = false;
        }

        uint8_t level = senseIR();

        for (int i = 0; i < LEDS_PER_CELL; i++) {
            if (m_flames[i].fuel > 0) {
                uint8_t burn = burn_rate; // random8(burn_rate);
                m_flames[i].fuel = qsub8(m_flames[i].fuel, burn);
                m_flames[i].heat = qadd8(m_flames[i].heat, burn);
            } else {
                uint8_t cool = 20;
                m_flames[i].heat = qsub8(m_flames[i].heat, cool);
            }
            
            byte colorindex = scale8( m_flames[i].heat, 170);
            setLEDHue( i, colorindex);

            if (m_flames[i].heat < 10) {
                uint8_t prob = map(level, 0, 255, sparking, 5);
                if ( random8() < prob ) {
                    uint8_t max_fuel = map(level, 0, 255, 150, 0);
                    uint8_t more_fuel = random8(max_fuel) + 100;
                    m_flames[i].fuel += more_fuel;
                }
            }
        }
    }

    // --- Surface patterns ---------------

    CRGB getSurfaceColor(int x, int y, uint8_t scaledown)
    {
        if (x < 0 || x >= SURFACE_WIDTH)  return CRGB::Black;
        if (y < 0 || y >= SURFACE_HEIGHT) return CRGB::Black;
        uint8_t hue = g_Surface[x][y];
        if (hue == 255) return CRGB::Black;
        CRGB color = ColorFromPalette(m_palette, hue);
        color %= scaledown;
        return color;
    }

    void SurfacePattern()
    {
        if (m_new_pattern) {
            m_palette = RainbowColors_p;
            m_new_pattern = false;
        }

        for (int i = 0; i < LEDS_PER_CELL; i++) {
            // -- Compute the coordinates of this LED
            SurfaceSample & s = g_SurfaceSamples[i];

            // -- Compute the absolute position of this LED on the surface
            int x = m_left + s.dx;
            int y = m_top + s.dy;
    
            // -- Look up the surface color and set the LED
            CRGB color = getSurfaceColor(x, y, s.top_left_part);

            if (s.top_right_part > 0) {
                color += getSurfaceColor(x+1, y, s.top_right_part);
            }

            if (s.bottom_left_part > 0) {
                color += getSurfaceColor(x, y+1, s.bottom_left_part);
            }

            if (s.bottom_right_part > 0) {
                color += getSurfaceColor(x+1, y+1, s.bottom_right_part);
            }
            
            setLED(i, color);
        }
    }
};

// === Surfaces =============================================================

void initializeSurface()
{
    // -- Precompute LED locations for surface view
    computePixelOffsets();

    for (int x = 0; x < SURFACE_WIDTH; x++) {
        for (int y = 0; y < SURFACE_HEIGHT; y++) {
            g_Surface[x][y] = 255;
        }
    }
}

bool setSurface(int x, int y, uint8_t val)
{
    if (x >= 0 and x < SURFACE_WIDTH and y >= 0 and y < SURFACE_HEIGHT) {
        g_Surface[x][y] = val;
        return true;
    } else {
        return false;
    }
}

bool mixSurface(int x, int y, uint8_t val)
{
    if (x >= 0 and x < SURFACE_WIDTH and y >= 0 and y < SURFACE_HEIGHT) {
        uint8_t old_val = g_Surface[x][y];
        if (old_val != 255) {
            val = (val + old_val)/2;
        }
        g_Surface[x][y] = val;
        return true;
    } else {
        return false;
    }
}

uint8_t getSurface(int x, int y)
{
    if (x >= 0 and x < SURFACE_WIDTH and y >= 0 and y < SURFACE_HEIGHT) {
        return g_Surface[x][y];
    } else {
        return 0;
    }
}

uint16_t g_f_center_x = SURFACE_WIDTH/2;
uint16_t g_f_center_y = SURFACE_HEIGHT/2;
uint8_t g_val = 128;

void ComputeCenterOfMass()
{
    uint32_t total_x = 0;
    uint32_t total_y = 0;
    uint32_t total_weight = 0;
    uint8_t m = 255;

    for (int i = 0; i < NUM_CELLS; i++) {
        Cell * cell = g_Cells[i];
        uint32_t x = cell->getCenterX();
        uint32_t y = cell->getCenterY();
        uint8_t v = cell->senseIR();
        if (v < m) {
            m = v;
        }
        
        uint32_t val = (256 - v)/16;
        total_x += x * val;        
        total_y += y * val;
        total_weight += val;
    }

    if (total_weight > 5) {
        // Serial.println(total_weight);
        g_f_center_x = total_x / total_weight;
        g_f_center_y = total_y / total_weight;
        g_val = m;
    }
}

void ComputeCenter(bool decay)
{
    uint16_t new_center_x = 0;
    uint16_t new_center_y = 0;
    uint8_t m = 255;
    for (int i = 0; i < NUM_CELLS; i++) {
        Cell * cell = g_Cells[i];
        uint32_t x = cell->getCenterX();
        uint32_t y = cell->getCenterY();
        uint8_t val = cell->senseIR();
        if (val < m) {
            m = val;
            new_center_x = cell->getCenterX();
            new_center_y = cell->getCenterY();
        }
    }
    if (m < 250) {
        if (decay) {
            g_f_center_x = (g_f_center_x + new_center_x)/2;
            g_f_center_y = (g_f_center_y + new_center_y)/2;
        } else {
            g_f_center_x = new_center_x;
            g_f_center_y = new_center_y;
        }
        g_val = m;
    }
}

void CenterSurface()
{
    int center_x;
    int center_y;
    ComputeCenter(true);
    center_x = g_f_center_x; // fixed_to_int(g_f_center_x);
    center_y = g_f_center_y; // fixed_to_int(g_f_center_y);
    // Serial.print(center_x); Serial.print(" "); Serial.println(center_y);

    for (int x = 0; x < SURFACE_WIDTH; x++) {
        for (int y = 0; y < SURFACE_HEIGHT; y++) {
            g_Surface[x][y] = 255;
        }
    }

    int size = 15 - g_val/20;

    int left = center_x - size;
    int right = center_x + size;
    int top = center_y - size;
    int bottom = center_y + size;

    for (int i = left; i < right; i++) {
        setSurface(i, top, g_val);
        setSurface(i, bottom, g_val);
    }
    
    for (int j = top; j < bottom; j++) {
        setSurface(left, j, g_val);
        setSurface(right, j, g_val);
    }

    /*
    for (int x = 0; x < SURFACE_WIDTH; x++) {
        for (int y = 0; y < SURFACE_HEIGHT; y++) {
            int diff_x = (x - center_x);
            int diff_y = (y - center_y);
            uint16_t dist2 = diff_x * diff_x + diff_y * diff_y;
            if (dist2 < 150 - (g_val/2)) {
                g_Surface[x][y] = g_val;
            } else {
                g_Surface[x][y] = 255;
            }
        }
    }
    */
}

class Ripple
{
private:
    bool is_on;
    uint16_t f_center_x;
    uint16_t f_center_y;
    uint8_t  radius;
    uint8_t  color;
    int num_visible;

public:
    Ripple() : is_on(false),
               f_center_x(0),
               f_center_y(0),
               radius(0), 
               color(128)
    {}

    bool isOn() { return is_on; }

    void init(uint16_t f_x, uint16_t f_y, uint8_t c)
    {
        is_on = true;
        f_center_x = f_x;
        f_center_y = f_y;
        radius = 1;
        color = c;
    }

    void drawOne(uint16_t f_x, uint16_t f_y)
    {
        int x = f_x; // fixed_to_int(f_x);
        int y = f_y; // fixed_to_int(f_y);
        if (mixSurface(x, y, color)) {
            num_visible++;
        }
    }

    void draw()
    {
        if (is_on) {
            num_visible = 0;
            // int increment = 64/radius;
            for (int angle = 0; angle < 64; angle += 4) {
                // -- Values 0 -- 255
                uint8_t x8 = cos8(angle) - 128;
                uint8_t y8 = sin8(angle) - 128;
                
                uint16_t f_delta_x = (x8 * radius)/128;
                uint16_t f_delta_y = (y8 * radius)/128;
                drawOne(f_center_x  + f_delta_x, f_center_y + f_delta_y);
                drawOne(f_center_x  - f_delta_x, f_center_y + f_delta_y);
                drawOne(f_center_x  + f_delta_x, f_center_y - f_delta_y);
                drawOne(f_center_x  - f_delta_x, f_center_y - f_delta_y);
                
            }
        }
 
        radius++;
        if (radius > 50 || num_visible == 0) {
            is_on = false;
        }
    }
};

#define NUM_RIPPLES 8
Ripple g_ripples[NUM_RIPPLES];
uint32_t g_last_new = 0;
int g_old_center_x = 0;
int g_old_center_y = 0;

void RippleSurface()
{
    for (int x = 0; x < SURFACE_WIDTH; x++) {
        for (int y = 0; y < SURFACE_HEIGHT; y++) {
            g_Surface[x][y] = 255;
        }
    }

    bool make_new = false;
    uint32_t cur = millis();
    if (cur - g_last_new > 150) {
        g_last_new = cur;
        ComputeCenter(false);
        int center_x = g_f_center_x; // fixed_to_int(g_f_center_x);
        int center_y = g_f_center_y; // fixed_to_int(g_f_center_y);
        if ((center_x != g_old_center_x) || (center_y != g_old_center_y)) {
            make_new = true;
            g_old_center_x = center_x;
            g_old_center_y = center_y;
        }
    }

    for (int i = 0; i < NUM_RIPPLES; i++) {
        if (g_ripples[i].isOn()) {
            g_ripples[i].draw();
        } else {
            if (make_new) {
                g_ripples[i].init(g_f_center_x, g_f_center_y, g_val);
                make_new = false;
            }
        }
    }
}

void AttractorSurface()
{
    
}

void DiffusionSurface()
{
    
}

void MirrorSurface()
{
    
}


// === Main logic ===========================================================

/** Calibrate the IR sensors
 *
 *  Read four values from each sensor, separated by 100ms. Take the
 *  average to be the max, which is used in Cell::senseIR to map all
 *  IR readings to a canonical range.
 */
void calibrate()
{
    uint16_t total_ir[NUM_CELLS];

    for (int i = 0; i < NUM_CELLS; i++) total_ir[i] = 0;

    for (int rounds = 0; rounds < 4; rounds++) {
        for (int i = 0; i < NUM_CELLS; i++) {
            uint16_t raw = g_Cells[i]->rawIR();
            total_ir[i] += raw;
        }
        delay(100);
    }

    for (int i = 0; i < NUM_CELLS; i++) {
        g_Cells[i]->setIRMax(total_ir[i] / 4);
        g_Cells[i]->setIRMin(140);
        CRGB gr = CRGB::Green;
        g_Cells[i]->setAllLEDs(gr);
        FastLED.show();
        delay(10);
    }
}

void initialize()
{
    // -- Create all the cell objects
    if (ONE_CELL_MODE) {
        CellMapEntry one = {0, 0, 0, 0};
        g_Cells[0] = new Cell(one);
    } else {
        for (int i = 0; i < NUM_CELLS; i++) {
            g_Cells[i] = new Cell(g_CellMap[i]);
        }
    }
    
    // -- Calibrate the IR sensors
    calibrate();

    // -- Initialize the surface view
    initializeSurface();
}

void changeToPattern(Mode newmode)
{
    g_Mode = newmode;

    for (int i = 0; i < NUM_CELLS; i++) {
        g_Cells[i]->newPattern();
    }

    Serial.print("New pattern ");
    Serial.println(newmode);
}

uint32_t last_change = 0;

void setup()
{
    delay(500);

    // -- Set up the pins
    if (ONE_CELL_MODE) {
        pinMode(LED_PIN, OUTPUT);
        pinMode(IR_INPUT_PIN, INPUT);
    } else {
        /*
        pinMode(LED_PIN_1, OUTPUT);
        pinMode(LED_PIN_2, OUTPUT);
        pinMode(LED_PIN_3, OUTPUT);
        */
        pinMode(17,OUTPUT);
        pinMode(16,OUTPUT);
        pinMode(4,OUTPUT);
        pinMode(2,OUTPUT);
        pinMode(15,OUTPUT);
        pinMode(12,OUTPUT);
    
        for (int i = 0; i < 4; i++) {
            pinMode(IR_INPUTS[i], INPUT);
        }

        pinMode(IR_CHANNEL_BIT_0, OUTPUT);
        pinMode(IR_CHANNEL_BIT_1, OUTPUT);
        pinMode(IR_CHANNEL_BIT_2, OUTPUT);
        pinMode(IR_CHANNEL_BIT_3, OUTPUT);
    }

    Serial.begin(115200);
    delay(200);

    // -- Add all the LEDs
    LED_setup();
    FastLED.setBrightness(g_Brightness);

    // -- Initialize the cells and calibrate
    
    fill_solid(g_LEDs, NUM_LEDS, CRGB::Yellow);
    FastLED.show();
    delay(1000);

    initialize();
    delay(500);

    fill_solid(g_LEDs, NUM_LEDS, CRGB::Black);
    FastLED.show();
    delay(1000);

    Serial.println();
    Serial.println("Ready...");
    last_change = millis();
}

uint32_t g_total_time = 0;
uint32_t g_frame_count = 0;

adel waitbutton(int pin)
{
   abegin:
   while (1) {
      await (digitalRead(pin) == HIGH);
      adelay (50);
      if (digitalRead(pin) == HIGH) {
         await (digitalRead(pin) == LOW);
         afinish;
      }
   }
   aend;
}

adel changemode()
{
    abegin:
    while (1) {
        andthen( waitbutton( MODE_PIN ) );

        g_Mode = Mode((g_Mode + 1) % NUM_MODES);
        changeToPattern(g_Mode);
        /*
        if (g_Mode == SolidMode)         changeToPattern(ConfettiMode);
        else if (g_Mode == ConfettiMode) changeToPattern(SpinnerMode);
        else if (g_Mode == SpinnerMode)  changeToPattern(GearMode);
        else if (g_Mode == GearMode)     changeToPattern(FireMode);
        else if (g_Mode == FireMode)     changeToPattern(SolidMode);
        */
    }
    
    aend;
}
    
adel table()
{
    uint32_t start;
    uint32_t end;
    bool is_surface;
  abegin:

    while (1) {
        start = millis();
        is_surface = (g_Mode >= SurfaceMode);

        if (is_surface) {
            if (g_Mode == SurfaceMode)   CenterSurface();
            if (g_Mode == RippleMode)    RippleSurface();
            if (g_Mode == DiffusionMode) DiffusionSurface();
            if (g_Mode == AttractorMode) AttractorSurface();
            if (g_Mode == MirrorMode)    MirrorSurface();
        }

        // -- Sense the IR and render the pattern
        for (int i = 0; i < NUM_CELLS; i++) {
            if (g_Mode == SolidMode)    g_Cells[i]->SolidPattern();
            if (g_Mode == ConfettiMode) g_Cells[i]->ConfettiPattern();
            if (g_Mode == SpinnerMode)  g_Cells[i]->SpinnerPattern(400, 8000);
            if (g_Mode == GearMode)     g_Cells[i]->GearPattern(400, 8000);
            if (g_Mode == FireMode)     g_Cells[i]->FirePattern(30, 50);
            if (is_surface)             g_Cells[i]->SurfacePattern();
        }

        if (g_Cycle) {
            // -- Periodically switch to a different mode
            uint32_t cur_time = millis();
            if (cur_time - last_change > TIME_PER_PATTERN) {
                last_change = cur_time;
                if (g_Mode == SolidMode)         changeToPattern(ConfettiMode);
                else if (g_Mode == ConfettiMode) changeToPattern(SpinnerMode);
                else if (g_Mode == SpinnerMode)  changeToPattern(GearMode);
                else if (g_Mode == GearMode)     changeToPattern(FireMode);
                else if (g_Mode == FireMode)     changeToPattern(SolidMode);
            }
        }
    
        FastLED.show();
        end = millis();
        g_total_time += (end - start);
        g_frame_count++;

    /** Just for testing: compute the time to render a frame
    if (g_frame_count == 40) {
        float fps = ((float) g_total_time) / ((float) g_frame_count);
        Serial.print("ms per frame: ");
        Serial.println(fps);
        g_frame_count = 0;
        g_total_time = 0;
    }
    */
    
        adelay(1000/FRAMES_PER_SECOND);
    }
    
    aend;
}

void loop()
{
    arepeat( table() );
    arepeat( changemode() );
}
