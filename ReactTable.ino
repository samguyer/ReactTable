#include <FastLED.h>

// === Pattern mode =========================================================

/** Mode
 *  This code supports four different animations. You can choose one of them, 
 *  or set "cycle" to true to cycle through all of them at some set interval.
 */
enum Mode { SolidMode, ConfettiMode, SpinnerMode, GearMode, FireMode, SurfaceMode, AttractorMode };

// -- Mode choice
Mode g_Mode = SolidMode;

// -- To cycle modes, set cycle to true and choose an interval (in milliseconds)
bool g_Cycle = false;
const int TIME_PER_PATTERN = 15000;

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
int IR_INPUTS[] = { 27, 33, 34, 35 };

/** Cell configuration
 *
 * My table has 61 cells, with 12 WS2812 LEDs per cell. The LEDs are
 * chained together to form a single, logical strip.
 */
#if ONE_CELL_MODE
#define NUM_CELLS 1
#else
#define NUM_CELLS 61
#endif

#define LEDS_PER_CELL 12
#define NUM_LEDS (NUM_CELLS * LEDS_PER_CELL)

/** LED strip configuration 
 *
 * To improve performance, I broke the strip into three segments, each
 * one attached to its own pin. Make sure the wiring matches the pin
 * configuration and the number of LEDs on each pin.
 * 
 */
#define LED_PIN_1 17
#define NUM_LEDS_1 (22 * LEDS_PER_CELL)

#define LED_PIN_2 16
#define NUM_LEDS_2 (22 * LEDS_PER_CELL)

#define LED_PIN_3  4
#define NUM_LEDS_3 (17 * LEDS_PER_CELL)

/** Default brightness */
uint8_t g_Brightness = 50;

/** Kind of LEDs */
#define COLOR_ORDER GRB
#define CHIPSET     WS2812

/** Animation speed */
#define FRAMES_PER_SECOND 30

/** Storage for LEDs */
CRGB g_LEDs[NUM_LEDS];

// === Cell mapping =========================================================

/** Cell mapping
 *
 * This data structure connects together the IR input information with
 * the LED information. It holds the IR input and the index of the
 * LED ring for that cell. Both values are between 0 and NUM_CELLS
 *
 * The reason we need this mapping is that the IR inputs are not wired
 * in the same order as the LED rings. I probably could have done it
 * that way, but I felt like it was easier to make the wiring simple
 * and fix it in software.
*/

struct CellMapEntry
{
    uint8_t    m_ir_index;
    uint16_t   m_ring_index;
    uint8_t    m_x;
    uint8_t    m_y;
};

CellMapEntry g_CellMap[] = {
    {  0, 60, 0, 0 }, {  1, 49, 2, 0 }, {  2, 38, 4, 0 }, {  3, 27, 6, 0 }, {  4, 16, 8, 0 }, {  5, 5, 10, 0 },
    {  6, 54, 1, 1 }, {  7, 43, 3, 1 }, {  8, 32, 5, 1 }, {  9, 21, 7, 1 }, { 10, 10, 9, 1 },
    { 11, 59, 0, 2 }, { 12, 48, 2, 2 }, { 13, 37, 4, 2 }, { 14, 26, 6, 2 }, { 15, 15, 8, 2 }, { 16, 4, 10, 2 },
    { 17, 53, 1, 3 }, { 18, 42, 3, 3 }, { 19, 31, 5, 3 }, { 20, 20, 7, 3 }, { 21,  9, 9, 3 },
    { 22, 58, 0, 4 }, { 23, 47, 2, 4 }, { 24, 36, 4, 4 }, { 25, 25, 6, 4 }, { 26, 14, 8, 4 }, { 27, 3, 10, 4 },
    { 28, 52, 1, 5 }, { 29, 41, 3, 5 }, { 30, 30, 5, 5 }, { 31, 19, 7, 5 }, { 32,  8, 9, 5 },
    { 33, 57, 0, 6 }, { 34, 46, 2, 6 }, { 35, 35, 4, 6 }, { 36, 24, 6, 6 }, { 37, 13, 8, 6 }, { 38, 2, 10, 6 },
    { 39, 51, 1, 7 }, { 40, 40, 3, 7 }, { 41, 29, 5, 7 }, { 42, 18, 7, 7 }, { 43,  7, 9, 7 },
    { 44, 56, 0, 8 }, { 45, 45, 2, 8 }, { 46, 34, 4, 8 }, { 47, 23, 6, 8 }, { 48, 12, 8, 8 }, { 49, 1, 10, 8 },
    { 50, 50, 1, 9 }, { 51, 39, 3, 9 }, { 52, 28, 5, 9 }, { 53, 17, 7, 9 }, { 54,  6, 9, 9 },
    { 55, 55, 0, 10}, { 56, 44, 2, 10}, { 57, 33, 4, 10}, { 58, 22, 6, 10}, { 59, 11, 8, 10}, { 60, 0, 10, 10 }
};

// === Single surface view ==================================================

/** Cell coordinate system
 *
 * The cells are arranged into 11 rows and 11 columns. The coordinates
 * provided in the CellInfo below should be in the cell coordinates.
 * The surface coordinates are computed from that.
 */
#define CELL_COLS 11
#define CELL_ROWS 11

/** Surface coordinate system
 *
 * The surface is a logical 2-D grid scaled up from the actual cell
 * coordinate system. In surface mode, the LEDs sample their values
 * from this single grid, allowing the entire table to be treated as a
 * single image. My table has a surface 65 units wide and 39 units tall.
 */
#define SURFACE_WIDTH_SCALE  5
#define SURFACE_HEIGHT_SCALE 3
#define SURFACE_WIDTH  ((CELL_COLS+2) * SURFACE_WIDTH_SCALE)
#define SURFACE_HEIGHT ((CELL_ROWS+2) * SURFACE_HEIGHT_SCALE)

/** Surface
 * 
 * Fill this 2-D grid with values to display a single image.
 */
uint8_t  g_Surface[SURFACE_WIDTH][SURFACE_HEIGHT];

/** Float to fixed
 *
 * Utility function to convert a float to a 16-bit fixed-precision
 * number (8 bits for the whole number, 8 bits for the fraction).
 */
uint16_t float_to_fixed(float v)
{
    uint8_t whole = (uint8_t) v;
    uint8_t frac = (uint8_t) (256.0 * (v - ((float)whole)));
    uint16_t result = whole;
    result <<= 8;
    result |= frac;
    return result;
}

/** Ring coordinate mapping
 *
 * Precompute the offsets for each LED in a ring relative to the
 * center.  The values are 16-bit fixed-precision numbers in the
 * surface coordinate system.
 */

uint16_t g_pixel_x[LEDS_PER_CELL];
uint16_t g_pixel_y[LEDS_PER_CELL];

void computePixelOffsets()
{
    for (int i = 0; i < LEDS_PER_CELL; i++) {
        // -- Divide the ring into 12 equal angles
        float frac = ((float) i) / ((float) LEDS_PER_CELL);
        float angle = frac * 3.1415926535897 * 2.0;
        
        // -- X and Y along a ring of diameter 4
        //    These values will serve as the top left corner of a box
        //    that samples the underlying image.
        float x = (cos(angle) + 1.0) * 2.0;
        float y = (sin(angle) + 1.0) * 2.0;

        // -- Convert to fixed point values
        g_pixel_x[i] = float_to_fixed(x);
        g_pixel_y[i] = float_to_fixed(y);
    }        
}

// === Cells ================================================================

class Cell;
Cell * g_Cells[NUM_CELLS];

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
    uint8_t         m_ring_x;
    uint8_t         m_ring_y;
    uint8_t         m_surface_x;
    uint8_t         m_surface_y;

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
          m_ring_x(info.m_x),
          m_ring_y(info.m_y),
          m_position(0),
          m_new_pattern(true)
    {
        // -- Precompute the IR selector signal
        m_ir_channel_selector[0] = (m_ir_channel & 0x1) ? HIGH : LOW;
        m_ir_channel_selector[1] = (m_ir_channel & 0x2) ? HIGH : LOW;
        m_ir_channel_selector[2] = (m_ir_channel & 0x4) ? HIGH : LOW;
        m_ir_channel_selector[3] = (m_ir_channel & 0x8) ? HIGH : LOW;

        // -- Compute the top left corner of the ring in surface
        //    coordinates.
        m_surface_x = ((m_ring_x + 1) * SURFACE_WIDTH_SCALE);
        m_surface_y = ((m_ring_y + 1) * SURFACE_HEIGHT_SCALE);
    }

    // ----- Getters --------

    uint16_t getRingX() const { return m_ring_x; }
    uint16_t getRingY() const { return m_ring_y; }
    int getSurfaceX() const { return m_surface_x; }
    int getSurfaceY() const { return m_surface_y; }

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

    CRGB getSurfaceColor(int x, int y)
    {
        if (x < 0 || x >= SURFACE_WIDTH)  return CRGB::Black;
        if (y < 0 || y >= SURFACE_HEIGHT) return CRGB::Black;
        uint8_t hue = g_Surface[x][y];
        if (hue == 255) return CRGB::Black;
        return ColorFromPalette(m_palette, hue);
    }

    void setSurfacePixel(int led_index)
    {
        // -- Compute the coordinates of this LED
        uint16_t x = m_ring_x + g_pixel_x[led_index];
        uint16_t y = m_ring_y + g_pixel_y[led_index];

        // -- Get the coordinates of the top left pixel
        uint16_t x_whole = x >> 8;
        uint16_t y_whole = y >> 8;

        // -- Get the fractional part, scale down to 0-15
        uint8_t x_frac = (x & 0xFF) >> 4;
        uint8_t y_frac = (y & 0xFF) >> 4;

        // -- Sample up to four pixels

        // -- Top left
        CRGB color = getSurfaceColor(x_whole, y_whole);
        uint8_t frac_TL = ((0x10 - x_frac) * (0x10 - y_frac)) - 1;
        color.nscale8_video(frac_TL);

        if (x_frac > 0) {
            // -- LED crosses into the adjacent pixel to the right
            CRGB color_TR = getSurfaceColor(x_whole + 1, y_whole);
            uint8_t frac_TR = x_frac * (0x10 - y_frac);
            color_TR.nscale8_video(frac_TR);
            color += color_TR;
        }

        if (y_frac > 0) {
            // -- LED crosses into the adjacent pixel below
            CRGB color_BL = getSurfaceColor(x_whole, y_whole + 1);
            uint8_t frac_BL = (0x10 - x_frac) * y_frac;
            color_BL.nscale8_video(frac_BL);
            color += color_BL;
        }

        if (x_frac > 0 and y_frac > 0) {
            // -- LED crosses both boundaries into the corner LED
            CRGB color_BR = getSurfaceColor(x_whole + 1, y_whole + 1);
            uint8_t frac_BR = x_frac * y_frac;
            color_BR.nscale8_video(frac_BR);
            color += color_BR;
        }

        setLED(led_index, color);
    }

    void SurfacePattern()
    {
        if (m_new_pattern) {
            m_palette = RainbowColors_p;
            m_new_pattern = false;
        }

        for (int i = 0; i < LEDS_PER_CELL; i++) {
            setSurfacePixel(i);
        }
    }

    void AttractorPattern(int center_x, int center_y)
    {
        if (m_new_pattern) {
            m_palette = RainbowColors_p;
            m_new_pattern = false;
        }
        int dist_x = m_ring_x - center_x;
        int dist_y = m_ring_y - center_y;
        int dist = dist_x * dist_x + dist_y * dist_y;
        int val = 255 - dist;
        setAllLEDsHue(val);
    }

};

// === Surfaces =============================================================

Cell * g_SurfaceCells[SURFACE_WIDTH][SURFACE_HEIGHT];
uint8_t g_Dist[20][20];
uint8_t g_nextSurface[SURFACE_WIDTH][SURFACE_HEIGHT];

/** Surface cells
 *  
 *  For each pixel on the surface, store the corresponding ring center (if 
 *  there is one), or NULL. This mapping allows us to quickly visit all of
 *  the elements of the surface and know which ones have sensors.
 */
void initializeSurface()
{
    // -- Precompute LED locations for surface view
    computePixelOffsets();

    for (int x = 0; x < SURFACE_WIDTH; x++) {
        for (int y = 0; y < SURFACE_HEIGHT; y++) {
            g_SurfaceCells[x][y] = NULL;
        }
    }

    for (int i = 0; i < NUM_CELLS; i++) {
        Cell * cell = g_Cells[i];
        int x = cell->getRingX();
        int y = cell->getRingY();
        g_SurfaceCells[cell->getRingX()][cell->getRingY()] = cell;
    }

    for (int dx = -10; dx < 10; dx++) {
        for (int dy = -10; dy < 10; dy++) {
            uint16_t dist = sqrt16(dx * dx + dy * dy);
            g_Dist[dx+10][dy+10] = dist;
        }
    }

    for (int x = 0; x < SURFACE_WIDTH; x++) {
        for (int y = 0; y < SURFACE_HEIGHT; y++) {
            g_nextSurface[x][y] = 0;
        }
    }
}

void setSurface(int x, int y, uint8_t val)
{
    if (x >= 0 and x < SURFACE_WIDTH and y >= 0 and y < SURFACE_HEIGHT) {
        g_Surface[x][y] = val;
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

void ComputeCenter(int & center_x, int & center_y)
{
    uint32_t total_x = 0;
    uint32_t total_y = 0;
    uint32_t total_weight = 0;

    for (int i = 0; i < NUM_CELLS; i++) {
        Cell * cell = g_Cells[i];
        uint32_t x = cell->getRingX();
        uint32_t y = cell->getRingY();
        uint32_t val = (255 - cell->senseIR())/16;
        total_x += x * val;        
        total_y += y * val;
        total_weight += val;
    }

    if (total_weight > 0) {
        center_x = total_x / total_weight;
        center_y = total_y / total_weight;
        /*
        Serial.print("Center ");
        Serial.print(center_x);
        Serial.print("  ");
        Serial.print(center_y);
        Serial.println();
        */
    }
}

void DiffusionSurface()
{
    for (int x = 0; x < SURFACE_WIDTH; x++) {
        for (int y = 0; y < SURFACE_HEIGHT; y++) {
            setSurface(x,y, x+y);
        }
    }
    /*
   for (int i = 0; i < NUM_CELLS; i++) {
        Cell * cell = g_Cells[i];
        int x = cell->getSurfaceX();
        int y = cell->getSurfaceY();
        uint8_t val = cell->senseIR();
        for (int j = -5; j < 5; j++) {
            for (int k = -5; k < 5; k++) {
                setSurface(x+j, y+k, val); 
            }
        }
    }
    */
    
    /*
    for (int i = 0; i < NUM_CELLS; i++) {
        Cell * cell = g_Cells[i];
        int x = cell->getSurfaceX();
        int y = cell->getSurfaceY();
        g_nextSurface[x][y] = 255 - cell->senseIR();
    }

    for (int x = 0; x < SURFACE_WIDTH; x++) {
        for (int y = 0; y < SURFACE_HEIGHT; y++) {
            if (g_nextSurface[x][y] == 0) {
                uint16_t neighbors = getSurface(x-1, y-1) + getSurface(x, y-1) + getSurface(x+1, y-1) +
                                     getSurface(x-1, y  )                      + getSurface(x+1, y  ) +
                                     getSurface(x-1, y+1) + getSurface(x, y+1) + getSurface(x+1, y+1);
                uint8_t newval = neighbors / 8;
                g_nextSurface[x][y] = newval;
            }
        }
    }

    for (int x = 0; x < SURFACE_WIDTH; x++) {
        for (int y = 0; y < SURFACE_HEIGHT; y++) {
            g_Surface[x][y] = g_nextSurface[x][y];
            g_nextSurface[x][y] = 0;
        }
    }
    */
}

class Particle
{
private:
    // -- Position
    float m_x;
    float m_y;
    
    // -- Velocity
    float m_vx;
    float m_vy;
    
    // -- Acceleration
    float m_ax;
    float m_ay;

public:

    Particle()
    : m_x(0.0),
      m_y(0.0),
      m_vx(0.0),
      m_vy(0.0),
      m_ax(0.0),
      m_ay(0.0)
    {}

    void reset()
    {
        // -- Pick a random place along the edge of the surface
        int pos = random16(SURFACE_WIDTH * 2 + SURFACE_HEIGHT * 2);
        if (pos < SURFACE_WIDTH) {
            // -- Top edge
            m_x = (float) pos;
            m_y = -1.0;
        } else if (pos < SURFACE_WIDTH + SURFACE_HEIGHT) {
            // -- Right edge
            m_x = (float) (SURFACE_WIDTH + 1);
            m_y = (float) (pos - SURFACE_WIDTH);
        } else if (pos < SURFACE_WIDTH + SURFACE_HEIGHT + SURFACE_WIDTH) {
            // -- Bottom edge
            m_x = (float) (pos - (SURFACE_WIDTH + SURFACE_HEIGHT));
            m_y = (float) (SURFACE_HEIGHT + 1);
        } else {
            // -- Left edge
            m_x = -1.0;
            m_y = (float) (pos - (SURFACE_WIDTH + SURFACE_HEIGHT + SURFACE_WIDTH));
        }
    }

    void attract(float attr_x, float attr_y, uint8_t val)
    {
        // -- Adjust acceleration based on distance to attractor
        float dx = (attr_x - m_x);
        float dy = (attr_y - m_y);

        m_ax = m_ax + (dx / 50.0);
        m_ay = m_ay + (dy / 50.0);
    }

    void move()
    {
        // -- Update the position using velocity
        m_x += m_vx;
        m_y += m_vy;

        // -- Update the velocity using acceleration
        m_vx += m_ax;
        m_vy += m_ay;

        // -- Reset the acceleration (which is computed by the attract method)
        m_ax = 0.0;
        m_ay = 0.0;

        if (m_vx < 0.5 and m_vy < 0.5) {
            // -- If a particle stops moving, start it over as a new one
            reset();
        }
    }

    void show()
    {
        // -- Color shows velocity
        int hue = (int) (m_vx * m_vx + m_vy * m_vy);

        int x = (int) m_x;
        int y = (int) m_y;
        setSurface(x, y, hue);
    }
};


Particle g_particles[50];

void ParticleSurface()
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
        pinMode(LED_PIN_1, OUTPUT);
        pinMode(LED_PIN_2, OUTPUT);
        pinMode(LED_PIN_3, OUTPUT);
    
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
    if (ONE_CELL_MODE) {
        FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(g_LEDs, NUM_LEDS).setCorrection( TypicalLEDStrip );
    } else {
        FastLED.addLeds<CHIPSET, LED_PIN_1, COLOR_ORDER>(g_LEDs, NUM_LEDS_1).setCorrection( TypicalLEDStrip );
        FastLED.addLeds<CHIPSET, LED_PIN_3, COLOR_ORDER>(g_LEDs, NUM_LEDS_1 + NUM_LEDS_2, NUM_LEDS_3).setCorrection( TypicalLEDStrip );
        FastLED.addLeds<CHIPSET, LED_PIN_2, COLOR_ORDER>(g_LEDs, NUM_LEDS_1, NUM_LEDS_2).setCorrection( TypicalLEDStrip );
    }
    
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
int g_center_x = 0;
int g_center_y = 0;

void loop()
{
    uint32_t start = millis();

    // if (g_Mode == SurfaceMode) DiffusionSurface();
    ComputeCenter(g_center_x, g_center_y);

    // -- Sense the IR and render the pattern
    for (int i = 0; i < NUM_CELLS; i++) {
        if (g_Mode == SolidMode)    g_Cells[i]->SolidPattern();
        if (g_Mode == ConfettiMode) g_Cells[i]->ConfettiPattern();
        if (g_Mode == SpinnerMode)  g_Cells[i]->SpinnerPattern(400, 8000);
        if (g_Mode == GearMode)     g_Cells[i]->GearPattern(400, 8000);
        if (g_Mode == FireMode)     g_Cells[i]->FirePattern(30, 50);
        if (g_Mode == SurfaceMode)  g_Cells[i]->SurfacePattern();
        if (g_Mode == AttractorMode)  g_Cells[i]->AttractorPattern(g_center_x, g_center_y);
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
    uint32_t end = millis();
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

    delay(1000/FRAMES_PER_SECOND);
}
