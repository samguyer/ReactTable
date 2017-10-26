#include <adel.h>
#include "SSD1306Wire.h"
#include <FastLED.h>

// ===== Global configuration ======================

#define CELL_COLS 24
#define CELL_ROWS 24

#define ROTARY_A_PIN 19
#define ROTARY_B_PIN 18

#define BUTTON_PIN 32

#define IR_CHANNEL_BIT_0 2  // 12
#define IR_CHANNEL_BIT_1 0  // 14
#define IR_CHANNEL_BIT_2 4  // 27
#define IR_CHANNEL_BIT_3 16 // 26

int IR_INPUT[] = { 35, 34, 39, 36 };

#define SDA_PIN 21
#define SCL_PIN 22

#define X_PIN 32
#define Y_PIN 33
#define Z_PIN 25

#define LED_PIN 17

#define NUM_CELLS 61
#define LEDS_PER_CELL 12
#define NUM_LEDS (NUM_CELLS * LEDS_PER_CELL)
#define FRAMES_PER_SECOND 60

// ===== Display ===================================

SSD1306Wire  display(0x3c, SDA_PIN, SCL_PIN); // display(Address, SDA, SCL)

// ===== LED grid information =============

CRGB g_LEDs[NUM_LEDS];

uint8_t g_Brightness = 50;

#define COLOR_ORDER GRB
#define CHIPSET     WS2812

// ===== Surface ==========================

template<int W, int H>
class IRSurface
{
private:

    uint8_t m_values[W][H];

public:

    IRSurface()
    {
        for (int i = 0; i < W; i++) {
            for (int j = 0; j < H; j++) {
                m_values[i][j] = 0;
            }
        }
    }

    /** Cubic Hermite for discrete space
     *
     *  Compute an interpolated value between B and C at a distance proportional
     *  to t. A and D are neighboring values that affect the curve near the edges
     *  (A is on the other side of B; D is on the other side of C):
     *
     *    A ....... B ....... C ....... D
     *              |<-t----->|
     *                 ^ 
     *                 | interpolated value
     *                 
     *  This version is discretized to make it more efficient: it can only
     *  interpolate 9 points in between. In other words, t should be a value
     *  in the range [0,8].
     *
     *  This choice makes certain computations fast. For example, instead of
     *  computing a * t * t * t (for values of t in [0,1]), we can compute
     *      a * (t/8) * (t/8) * (t/8)
     *    = a * t * t * t / (8 * 8 * 8)
     *    = (a * t * t * t) >> 9
     */

    uint8_t cubic8( uint8_t A, uint8_t B, uint8_t C, uint8_t D, uint16_t t)
    {
        uint32_t Do2 = D/2;
        uint32_t Ao2 = A/2;
        uint32_t a = -Ao2 + (3*B)/2 - (3*C)/2 + Do2;
        uint32_t b = A - (5*B)/2 + 2*C - Do2;
        uint32_t c = -Ao2 + C/2;
        uint32_t d = B;

        uint32_t at3 = (a * t * t * t) >> 9;
        uint32_t bt2 = (b * t * t) >> 6;
        uint32_t ct  = (c * t) >> 3;
        uint32_t r32 = at3 + bt2 + ct + d;
        return uint8_t(r32);
    }

    uint8_t getPixel(uint16_t X, uint16_t Y, int xdelta, int ydelta)
    {
        return 0;
    }

    uint8_t sampleBicubic(uint16_t x, uint16_t y)
    {
        uint16_t X = x >> 4;
        uint16_t Y = y >> 4;

        return 0;
    }
};


// ===== Patterns ====================================================

CRGBPalette16 g_palette[] = 
    { RainbowColors_p, 
      CloudColors_p,
      LavaColors_p,
      OceanColors_p,
      ForestColors_p,
      PartyColors_p,
      HeatColors_p };


class Cell;

class Pattern
{
friend class Cell;
    
protected:
    Cell * m_cell;
    
public:
    Pattern()
        : m_cell(0)
    {}

    virtual ~Pattern() {}
    virtual Pattern * clone() =0;
    virtual void render(uint8_t level, uint32_t delta_t) =0;

private:
    void setCell(Cell * cell) {
        m_cell = cell;
    }
};

class Cell
{
private:
    // -- Cell position
    int       m_x;
    int       m_y;
    
    // -- IR configuration
    int       m_ir_input;
    int       m_ir_channel;
    int       m_ir_max;
    int       m_ir_min;
    uint16_t  m_ir_history[8];
    int       m_ir_recent;
    int       m_ir;

    uint8_t   m_level;
    bool      m_decay;
    uint8_t   m_up_decay;
    uint8_t   m_down_decay;

    // -- LED ring information
    int             m_led_index;
    uint16_t        m_translate;
    CRGBPalette16   m_palette;

    // -- Visual pattern
    Pattern *       m_pattern;

    // -- Visual options
    uint8_t         m_fade_by;
    
public:
    Cell(int ir_input, int ir_channel, int X, int Y, int led_pos )
        : m_x(X),
          m_y(Y),
          m_ir_input(ir_input),
          m_ir_channel(ir_channel),
          m_ir_recent(0),
          m_decay(true),
          m_up_decay(4),
          m_down_decay(8),
          m_led_index(led_pos * LEDS_PER_CELL), 
          m_translate(0),
          m_palette(RainbowColors_p),
          m_pattern(0),
          m_fade_by(0)
    {
        calibrateIR();
    }

    // ----- Getters and setters -----

    // -- Set the palette explicitly
    void setPalette(CRGBPalette16 new_palette) {
        m_palette = new_palette;
    }

    // -- Set the palette by index
    void setPalette(int which) {
        if (which >= 0 and which < 6)  m_palette = g_palette[which];
        else m_palette = RainbowColors_p;
    }

    void setTranslation(uint16_t trans) { m_translate = trans; }
    uint16_t getTranslation(void) const { return m_translate; }

    // ----- Rendering methods -----

    // -- Get a reference to an LED based on its local index
    CRGB& getRawLED(int local_index) {
        if (local_index < 0) local_index = 0;
        if (local_index >= LEDS_PER_CELL) local_index = LEDS_PER_CELL - 1;
        return g_LEDs[m_led_index + local_index];
    }

    /* Set a pixel
     *  
     * The position is expressed as a 16-bit fixed-precision number that maps
     * to whatever number of actual LEDs there are in the strip/ring. This
     * function dithers the adjacent pixels to create a smooth transition.
     */
    void setPixel(uint16_t pos_in, uint8_t hue, uint8_t brightness = 255) {
        // -- Apply translation
        //    This might overflow, which supports LED rings
        uint16_t pos = pos_in + m_translate;
      
        // -- Get the color from the palette
        CRGB color = ColorFromPalette(m_palette, hue);
        if (brightness < 255) color.nscale8_video(brightness);

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
        uint8_t offsetB = 0xFF - offsetA;

        // -- Scale the brightness and assign the LEDs
        if (offsetA > 10) nblend(g_LEDs[m_led_index + indexA], color, offsetA);
        if (offsetB > 10) nblend(g_LEDs[m_led_index + indexB], color, offsetB);
    }

    /** Set an LED
     *  
     *  Set an LED value based on LED index rather than abstract position.
     *  Other than possible translation of coordinates, this should end
     *  up setting the color on a particular real LED device.
     */
    void setLED(uint8_t led, uint8_t hue, uint8_t brightness = 255) {
        /*
          CRGB color = ColorFromPalette(m_palette, hue);
          if (brightness < 255) color.nscale8_video(brightness);
          g_LEDs[m_led_index + led] = color;
        */
        uint16_t pos = ((((uint16_t) led) << 8) / LEDS_PER_CELL) << 8;
        setPixel(pos, hue, brightness);
    }

    /** Set all LEDs
     *  
     *  Set all the LEDs to a particular color.
     */
    void setAllLEDs(CRGB& color) {
        for (int i = 0; i < LEDS_PER_CELL; i++) {
            g_LEDs[m_led_index + i] = color;
        }
    }

    /** Set all LEDs
     *  
     *  Set all the LEDs to a particular hue.
     */
    void setAllLEDs(uint8_t hue, uint8_t brightness = 255) {
        CRGB color = ColorFromPalette(m_palette, hue);
        if (brightness < 255) color.nscale8_video(brightness);
        setAllLEDs(color);
    }

    /** Fade all LEDs
     *  
     */
    void fadeToBlackBy(uint8_t howmuch) {
        for (int i = 0; i < LEDS_PER_CELL; i++) {
            g_LEDs[m_led_index + i].fadeToBlackBy(howmuch);
        }
    }

    // ----- IR sensing -----

    /** Read raw IR value
     *
     *  Read from the channel assigned to this cell. 
     */
    int readIR()
    {
        uint8_t channel = m_ir_channel;

        // -- Select the channel
        if (channel & 0x1) digitalWrite(IR_CHANNEL_BIT_0, HIGH);
        else digitalWrite(IR_CHANNEL_BIT_0, LOW);

        if (channel & 0x2) digitalWrite(IR_CHANNEL_BIT_1, HIGH);
        else digitalWrite(IR_CHANNEL_BIT_1, LOW);

        if (channel & 0x4) digitalWrite(IR_CHANNEL_BIT_2, HIGH);
        else digitalWrite(IR_CHANNEL_BIT_2, LOW);

        if (channel & 0x8)digitalWrite(IR_CHANNEL_BIT_3, HIGH);
        else digitalWrite(IR_CHANNEL_BIT_3, LOW);

        // -- Finally, read the analog value
        int val = analogRead(IR_INPUT[m_ir_input]);
        return val;
    }

    /** Calibrate IR
     *
     *  Read a series of values, using the average value as the max
     */
    void calibrateIR()
    {
        // -- Calibrate the max distance (assume we boot up with nothing
        //    in front of the display)
        int acc = 0;
        for (int i = 0; i < 4; i++) {
            acc += readIR();
            delay(20);
        }

        m_ir_max = acc/4;
        m_ir_min = 140;

        for (int i = 0; i < 8; i++) {
            m_ir_history[i] = 0;
        }
        Serial.print(m_ir_channel); Serial.print("  "); Serial.println(m_ir_max);
    }
    
    uint8_t senseIR()
    {
        int raw = readIR();
        m_ir_history[m_ir_recent] = raw;
        m_ir_recent++;
        if (m_ir_recent >= 8) m_ir_recent = 0;
  
        uint32_t total = 0;
        total += m_ir_history[0];
        total += m_ir_history[1];
        total += m_ir_history[2];
        total += m_ir_history[3];
        total += m_ir_history[4];
        total += m_ir_history[5];
        total += m_ir_history[6];
        total += m_ir_history[7];
      
        m_ir = total >> 3;

        // -- Make sure the value makes sense
        if (m_ir < m_ir_min) m_ir = m_ir_min;
        if (m_ir > m_ir_max) m_ir = m_ir_max;

        // -- Map into the range 0-255
        int level = map(m_ir, m_ir_min, m_ir_max, 0, 255);

        //if (m_ir_channel == 3 && m_ir_input == 2) {
        //    Serial.println(level);
        //}

        return level;
    }

    uint8_t decayIR(uint8_t cur_level, uint32_t delta_t)
    {
        if (cur_level < m_level) {
            uint8_t new_level = qsub8(m_level, m_down_decay);
            if (cur_level < new_level) {
                m_level = new_level;
            } else {
                m_level = cur_level;
            }
        } else {
            if (cur_level > m_level) {
                uint8_t new_level = qadd8(m_level, m_up_decay);
                if (cur_level > new_level) {
                    m_level = new_level;
                } else {
                    m_level = cur_level;
                }
            }
        }
        return m_level;
    }

    // -------- Pattern management --------

    void setPattern(Pattern * pattern)
    {
        if (m_pattern)
            delete m_pattern;
            
        m_pattern = pattern;
        m_pattern->setCell(this);
    }
    
    void clonePattern(Pattern * prototype)
    {
        setPattern(prototype->clone());
    }
    
    // -------- Visual Rendering --------
    
    void render(uint32_t delta_t)
    {
        if (m_fade_by > 0) {
            fadeToBlackBy(m_fade_by);
        }

        uint8_t level = senseIR();
        if (m_decay) {
            level = decayIR(level, delta_t);
        }
        
        if (m_pattern)
            m_pattern->render(level, delta_t);
    }
};

class SolidPattern : public Pattern
{
public:
    SolidPattern()
        : Pattern()
    {}

    virtual Pattern * clone()
    {
        return new SolidPattern(*this);
    }

    virtual void render(uint8_t level, uint32_t delta_t)
    {
        if (level > 230) level = 230;
        m_cell->setAllLEDs(level);
    }
};

class DotPattern : public Pattern
{
private:
    int m_min_speed;
    int m_max_speed;
    uint16_t m_position;

public:
    DotPattern(int min_speed, int max_speed)
        : Pattern(),
          m_min_speed(min_speed),
          m_max_speed(max_speed),
          m_position(0)
    {}

    virtual Pattern * clone()
    {
        return new DotPattern(*this);
    }

    virtual void render(uint8_t level, uint32_t delta_t)
    {
        if (level > 230) level = 230;
        int speed = map(level, 0, 255, m_max_speed, m_min_speed);
        m_position += speed;
        m_cell->setPixel(m_position, level);
    }
};

class FirePattern : public Pattern
{ 
private:
    int m_sparking;
    byte m_heat[LEDS_PER_CELL];

public:
    FirePattern(int sparking)
        : Pattern(),
          m_sparking(sparking)
    {
        for (int i = 0; i < LEDS_PER_CELL; i++) {
            m_heat[i] = 0;
        }
        m_cell->setPalette(HeatColors_p);
        m_cell->setTranslation(random16());
    }

    virtual Pattern * clone()
    {
        return new FirePattern(*this);
    }

    void render(uint8_t level, uint32_t delta_t)
    { 
        int num_leds = LEDS_PER_CELL;
      
        // Temperature is in arbitrary units from 0 (cold black) to 255 (white hot).
      
        // COOLING: How much does the air cool as it rises?
        // Less cooling = taller flames.  More cooling = shorter flames.
        // Default 55, suggested range 20-100       
        int cooling = map(level, 0, 255, 20, 150);
            
        // Step 1.  Cool down every cell a little
        for( int i = 0; i < num_leds; i++) {
            m_heat[i] = qsub8( m_heat[i],  random8(0, ((cooling * 10) / num_leds) + 2));
        }
    
        // Step 2.  Heat from each cell drifts 'up' and diffuses a little
        for( int k= num_leds - 1; k >= 2; k--) {
            m_heat[k] = (m_heat[k - 1] + m_heat[k - 2] + m_heat[k - 2] ) / 3;
        }
      
        // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
        if( random8() < m_sparking ) {
            int y = random8(3);
            m_heat[y] = qadd8( m_heat[y], random8(100,150) );
        }
  
        // Step 4.  Map from heat cells to LED colors
        for( int j = 0; j < num_leds; j++) {
            // Scale the heat value from 0-255 down to 0-240
            // for best results with color palettes.
            byte colorindex = scale8( m_heat[num_leds - j - 1], 170);
            m_cell->setLED(j, colorindex);
        }
    }
};

class WavePattern : public Pattern
{
public:

    WavePattern()
        : Pattern()
    {}

    virtual Pattern * clone()
    {
        return new WavePattern(*this);
    }

    // ColorWavesWithPalettes by Mark Kriegsman: https://gist.github.com/kriegsman/8281905786e8b2632aeb
    // This function draws color waves with an ever-changing,
    // widely-varying set of parameters, using a color palette.
    void render(uint8_t level, uint32_t delta_t)
    {
        static uint16_t sPseudotime = 0;
        static uint16_t sLastMillis = 0;
        static uint16_t sHue16 = 0;
    
        // uint8_t sat8 = beatsin88( 87, 220, 250);
        uint8_t brightdepth = beatsin88( 341, 96, 224);
        uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
        uint8_t msmultiplier = beatsin88(147, 23, (64 - (level/4)) + 60);
    
        uint16_t hue16 = sHue16;//gHue * 256;
        uint16_t hueinc16 = beatsin88(113, 300, 1500);
    
        uint16_t ms = millis();
        uint16_t deltams = ms - sLastMillis ;
        sLastMillis  = ms;
        sPseudotime += deltams * msmultiplier;
        sHue16 += deltams * beatsin88( 400, 5, 9);
        uint16_t brightnesstheta16 = sPseudotime;
    
        for ( uint16_t i = 0 ; i < LEDS_PER_CELL; i++) {
            hue16 += hueinc16;
            uint8_t hue8 = hue16 / 256;
            uint16_t h16_128 = hue16 >> 7;
            if ( h16_128 & 0x100) {
                hue8 = 255 - (h16_128 >> 1);
            } else {
                hue8 = h16_128 >> 1;
            }
    
            brightnesstheta16  += brightnessthetainc16;
            uint16_t b16 = sin16( brightnesstheta16  ) + 32768;
    
            uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
            uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
            bri8 += (255 - brightdepth);
    
            uint8_t index = hue8;
            //index = triwave8( index);
            index = scale8( index, 240);

            m_cell->setLED(i, index, bri8);
            /*
              CRGB newcolor = ColorFromPalette(m_palette, index, bri8);
              nblend(getLED(i), newcolor, 128);
            */
        }
    }
};

class DiscoPattern : public Pattern
{
private:
    uint8_t m_range;
    uint8_t m_speed;
    
    uint8_t hue[LEDS_PER_CELL];
    uint8_t hue_level[LEDS_PER_CELL];
    uint8_t position[LEDS_PER_CELL];

public:
    DiscoPattern(int range, int speed)
        : Pattern(),
          m_range(range),
          m_speed(speed)
    {
        for (int i = 0; i < LEDS_PER_CELL; i++) {
            // -- Random hues
            hue[i] = random8(m_range);
            hue_level[i] = 0;
            // -- Random starting points on the curve
            position[i] = random8();
        }
    }

    virtual Pattern * clone()
    {
        return new DiscoPattern(*this);
    }

    void render(uint8_t level, uint32_t delta_t) 
    {
        uint8_t adjusted_level = lerp8by8(0, 240, level);
        for (int i = 0; i < LEDS_PER_CELL; i++) {
            // -- Compute brightness a position on cosine wave
            uint8_t prev_pos = position[i];
            position[i] = prev_pos + m_speed; // Should give about 1/2 second per cycle
            uint8_t cur_pos = position[i];
            uint8_t wave = cos8(cur_pos);
            uint8_t brightness = lerp8by8(20, 255, wave);

            // -- Hue is incremented to match the level
            uint8_t cur_hue_level = hue_level[i];
            if (cur_hue_level < adjusted_level) hue_level[i] += min(2, adjusted_level - cur_hue_level);
            if (cur_hue_level > adjusted_level) hue_level[i] = (cur_hue_level * 3 + adjusted_level) / 4;
        
            m_cell->setLED(i, hue[i] + hue_level[i], brightness);

            // -- At the end of each cycle, pick a new color
            if (prev_pos < 128 && cur_pos >= 128) {
                hue[i] = random8(m_range);
            }
        }
    }
};


// ===== Cells ====================================================

Cell * g_Cells[NUM_CELLS];

void init_cells()
{
    int index = 0;
    //                         IRin IRch X   Y  LED
    g_Cells[index++] = new Cell( 0,  0,  2,  1, 60 );
    g_Cells[index++] = new Cell( 0,  1,  6,  1, 49 );
    g_Cells[index++] = new Cell( 0,  2, 10,  1, 38 );
    g_Cells[index++] = new Cell( 0,  3, 14,  1, 27 );
    g_Cells[index++] = new Cell( 0,  4, 18,  1, 16 );
    g_Cells[index++] = new Cell( 0,  5, 22,  1,  5 );
    g_Cells[index++] = new Cell( 0,  6,  4,  2, 54 );
    g_Cells[index++] = new Cell( 0,  7,  8,  2, 43 );
    g_Cells[index++] = new Cell( 0,  8, 12,  2, 32 );
    g_Cells[index++] = new Cell( 0,  9, 16,  2, 21 );
    g_Cells[index++] = new Cell( 0, 10, 20,  2, 10 );
    g_Cells[index++] = new Cell( 0, 11,  2,  3, 59 );
    g_Cells[index++] = new Cell( 0, 12,  6,  3, 48 );
    g_Cells[index++] = new Cell( 0, 13, 10,  3, 37 );
    g_Cells[index++] = new Cell( 0, 14, 14,  3, 26 );
    g_Cells[index++] = new Cell( 0, 15, 18,  3, 15 );
    g_Cells[index++] = new Cell( 1,  0, 22,  3,  4 );
    g_Cells[index++] = new Cell( 1,  1,  4,  4, 53 );
    g_Cells[index++] = new Cell( 1,  2,  8,  4, 42 );
    g_Cells[index++] = new Cell( 1,  3, 12,  4, 31 );
    g_Cells[index++] = new Cell( 1,  4, 16,  4, 20 );
    g_Cells[index++] = new Cell( 1,  5, 20,  4,  9 );
    g_Cells[index++] = new Cell( 1,  6,  2,  5, 58 );
    g_Cells[index++] = new Cell( 1,  7,  6,  5, 47 );
    g_Cells[index++] = new Cell( 1,  8, 10,  5, 36 );
    g_Cells[index++] = new Cell( 1,  9, 14,  5, 25 );
    g_Cells[index++] = new Cell( 1, 10, 18,  5, 14 );
    g_Cells[index++] = new Cell( 1, 11, 22,  5,  3 );
    g_Cells[index++] = new Cell( 1, 12,  4,  6, 52 );
    g_Cells[index++] = new Cell( 1, 13,  8,  6, 41 );
    g_Cells[index++] = new Cell( 1, 14, 12,  6, 30 );
    g_Cells[index++] = new Cell( 1, 15, 16,  6, 19 );
    g_Cells[index++] = new Cell( 2,  0, 20,  6,  8 );
    g_Cells[index++] = new Cell( 2,  1,  2,  7, 57 );
    g_Cells[index++] = new Cell( 2,  2,  6,  7, 46 );
    g_Cells[index++] = new Cell( 2,  3, 10,  7, 35 );
    g_Cells[index++] = new Cell( 2,  4, 14,  7, 24 );
    g_Cells[index++] = new Cell( 2,  5, 18,  7, 13 );
    g_Cells[index++] = new Cell( 2,  6, 22,  7,  2 );
    g_Cells[index++] = new Cell( 2,  7,  4,  8, 51 );
    g_Cells[index++] = new Cell( 2,  8,  8,  8, 40 );
    g_Cells[index++] = new Cell( 2,  9, 12,  8, 29 );
    g_Cells[index++] = new Cell( 2, 10, 16,  8, 18 );
    g_Cells[index++] = new Cell( 2, 11, 20,  8,  7 );
    g_Cells[index++] = new Cell( 2, 12,  2,  9, 56 );
    g_Cells[index++] = new Cell( 2, 13,  6,  9, 45 );
    g_Cells[index++] = new Cell( 2, 14, 10,  9, 34 );
    g_Cells[index++] = new Cell( 2, 15, 14,  9, 23 );
    g_Cells[index++] = new Cell( 3,  0, 18,  9, 12 );
    g_Cells[index++] = new Cell( 3,  1, 22,  9,  1 );
    g_Cells[index++] = new Cell( 3,  2,  4, 10, 50 );
    g_Cells[index++] = new Cell( 3,  3,  8, 10, 39 );
    g_Cells[index++] = new Cell( 3,  4, 12, 10, 28 );
    g_Cells[index++] = new Cell( 3,  5, 16, 10, 17 );
    g_Cells[index++] = new Cell( 3,  6, 20, 10,  6 );
    g_Cells[index++] = new Cell( 3,  7,  2, 11, 55 );
    g_Cells[index++] = new Cell( 3,  8,  6, 11, 44 );
    g_Cells[index++] = new Cell( 3,  9, 10, 11, 33 );
    g_Cells[index++] = new Cell( 3, 10, 14, 11, 22 );
    g_Cells[index++] = new Cell( 3, 11, 18, 11, 11 );
    g_Cells[index++] = new Cell( 3, 12, 22, 11,  0 );
    assert( index == NUM_CELLS);
}

/*
typedef CellWrapper<Fade<40,
                         ColorDot<400, 4000, 
                                  DecayLevel< 2, 12,
                                              IRSense< BasePattern > > > > > Cell1;

typedef CellWrapper<Fire< 100, 
                          //DecayLevel< 1, 16,
                          IRSense< BasePattern > > > Cell2;

typedef CellWrapper< Fade<40, 
                          ColorSolid< 
                              DecayLevel<1, 4, 
                                         IRSense< BasePattern > > > > > Cell3;

typedef CellWrapper< ColorSolid< DecayLevel<4, 10, IRSense< BasePattern > > > > Cell4;

typedef CellWrapper< Disco<60, 3, 
                           DecayLevel<1, 4,
                                      IRSense< BasePattern > > > > Cell5;


Cell * g_cells[NUM_CELLS];
*/

void set_all_patterns(Pattern * prototype)
{
    for (int i = 0; i < NUM_CELLS; i++) {
        g_Cells[i]->clonePattern( prototype );
    }
}   

// ===== Buttons and controls =============

adel waitTurn(int pinA, int pinB)
{
 abegin:
    while (1) {
        if ( digitalRead(pinA) == HIGH) {
            await( digitalRead(pinA) == LOW  );
            if ( digitalRead(pinB) == HIGH) {
                await( digitalRead(pinB) == LOW  );
                // if ( digitalRead(pinA) == LOW)
                afinish;
            }
        }
        adelay(1);
    }

    aend;
}

adel rotary(int * pDelta)
{
 abegin:
    (*pDelta) = 0;
    auntil( waitTurn(ROTARY_A_PIN, ROTARY_B_PIN), 
            waitTurn(ROTARY_B_PIN, ROTARY_A_PIN) ) {
        (*pDelta) = 1;
    } else {
        (*pDelta) = -1;
    }
    aend;
}
  
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

adel adjustments()
{
    int delta;
    int val;
    
 abegin:
    val = 0;
    
    while (1) {
        andthen( rotary( &delta ) );
        val += delta;
        Serial.print("Val = ");
        Serial.println(val);
    }

    aend;
}

adel changemode()
{
 abegin:
    while (1) {
        andthen( waitbutton (BUTTON_PIN) );
        Serial.println("Button");
    }
    aend;
}

// ===== Setup ============================

void setup()
{
    pinMode(LED_PIN, OUTPUT);

    pinMode(BUTTON_PIN, INPUT);
    pinMode(ROTARY_A_PIN, INPUT_PULLUP);
    pinMode(ROTARY_B_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    delay(200);
    
    // -- Set up the LED rings (as a giant strip)
    FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(g_LEDs, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(g_Brightness);
    
    fill_solid(g_LEDs, NUM_LEDS, CRGB::Black);
    FastLED.show();
    
    // -- Set up the OLED display
    
    display.init();
    
    // display.flipScreenVertically();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);

    for (int i = 0; i < 4; i++) {
        pinMode(IR_INPUT[i], INPUT);
    }

    pinMode(IR_CHANNEL_BIT_0, OUTPUT);
    pinMode(IR_CHANNEL_BIT_1, OUTPUT);
    pinMode(IR_CHANNEL_BIT_2, OUTPUT);
    pinMode(IR_CHANNEL_BIT_3, OUTPUT);

    init_cells();
    SolidPattern solid;
    set_all_patterns(&solid);
    
    Serial.println("READY!");
    display.drawString(10, 28, "Ready!");
    display.display();
    delay(2000);
}

adel render()
{
    uint32_t delta_time = 0;
    uint32_t last_time = 0;
    
 abegin:
    while (1) {
        delta_time = millis() - last_time;
        last_time = millis();
        for (int i = 0; i < NUM_CELLS; i++) {
            if (g_Cells[i]) {
                g_Cells[i]->render(delta_time);
            }
        }
        FastLED.show();
        adelay(1000/FRAMES_PER_SECOND);
    }
    aend;
}

void loop()
{
    // arepeat( adjustments() );
    // arepeat( changemode() );
    arepeat( render() );
}

