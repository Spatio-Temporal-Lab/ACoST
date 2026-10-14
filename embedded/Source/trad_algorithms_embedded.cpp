 /**
 * trad_algorithms_embedded.cpp
 *
 * Implementations of OPWTR, Dead Reckoning, SQUISH-E and VOLTCom
 * declared in trad_algorithms_embedded.h.
 */

#include "trad_algorithms_embedded.h"

// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T
//  1. OPWTR
// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T

void OpwtrState::init(double eps) {
    epsilon   = eps;
    win_start = 0;
    win_count = 0;
    has_key   = false;
    memset(&last_key, 0, sizeof(last_key));
}

bool OpwtrState::feed(const TradPoint &pt) {
    // ©¤©¤ Add new point to window ©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤
    if (win_count < OPWTR_WINDOW) {
        win[(win_start + win_count) % OPWTR_WINDOW] = pt;
        ++win_count;
    } else {
        // Window full: evict oldest before adding
        // (treated same as if SED failed ¡ª emit anchor, shift)
        last_key = win[win_start];
        has_key  = true;
        win_start = (win_start + 1) % OPWTR_WINDOW;
        --win_count;
        win[(win_start + win_count) % OPWTR_WINDOW] = pt;
        ++win_count;
        return true;
    }

    if (win_count < 3) return false;

    // ©¤©¤ OPWTR check: for all intermediate points between anchor and end,
    //   SED(anchor, end, intermediate) must be <= epsilon ©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤
    TradPoint &anchor = win[win_start];
    TradPoint &end    = win[(win_start + win_count - 1) % OPWTR_WINDOW];

    double eps2 = epsilon * epsilon;
    bool   ok   = true;
    for (int k = 1; k < win_count - 1 && ok; ++k) {
        TradPoint &mid = win[(win_start + k) % OPWTR_WINDOW];
        if (sed_deg2(anchor, end, mid) > eps2) {
            ok = false;
        }
    }

    if (!ok) {
        // Emit anchor, new anchor = the point JUST BEFORE end (index win_count-2)
        last_key = win[win_start];
        has_key  = true;

        // Shift window: keep from the point that caused violation
        // The violation was at intermediate point k; new anchor = win[win_start+k-1]
        // Simpler correct implementation: anchor advances by 1
        win_start = (win_start + 1) % OPWTR_WINDOW;
        --win_count;
        return true;
    }
    return false;
}

bool OpwtrState::flush() {
    if (win_count == 0) return false;
    // Emit the last anchor
    last_key = win[win_start];
    has_key  = true;
    win_start = (win_start + 1) % OPWTR_WINDOW;
    --win_count;
    return true;
}

// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T
//  2. Dead Reckoning
// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T

void DrState::init(double eps_m) {
    epsilon = eps_m;
    started = false;
    accum   = 0.0;
    has_key = false;
    memset(&prev,     0, sizeof(prev));
    memset(&last_key, 0, sizeof(last_key));
}

bool DrState::feed(const TradPoint &pt) {
    if (!started) {
        prev         = pt;
        anchor_angle = 0.0;
        accum        = 0.0;
        started      = true;
        // First point is always a key-point
        last_key = pt;
        has_key  = true;
        return true;
    }

    double dist  = haversine_m(prev.lat, prev.lon, pt.lat, pt.lon);
    double dlon  = pt.lon - prev.lon;
    double dlat  = pt.lat - prev.lat;
    double angle = std::atan2(dlat, dlon);  // radians, approximate heading

    accum += dist * std::sin(angle - anchor_angle);

    prev = pt;

    if (std::fabs(accum) > epsilon) {
        accum        = 0.0;
        anchor_angle = angle;
        last_key = pt;
        has_key  = true;
        return true;
    }
    return false;
}

bool DrState::flush() {
    if (!started) return false;
    last_key = prev;
    has_key  = true;
    return true;
}

// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T
//  3. SQUISH-E (fixed window)
// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T

void SquisheState::init(double eps) {
    epsilon = eps;
    count   = 0;
    has_key = false;
    memset(&last_key, 0, sizeof(last_key));
    for (int i = 0; i < SQUISHE_BUF; ++i) buf[i].valid = false;
}

void SquisheState::recalc_priorities() {
    // Compute SED priority for each interior point
    for (int k = 1; k < count - 1; ++k) {
        double s = sed_deg2(buf[k-1].pt, buf[k+1].pt, buf[k].pt);
        buf[k].priority = s;
    }
    if (count > 0) buf[0].priority        = 1e30;  // anchor never evicted
    if (count > 1) buf[count-1].priority  = 1e30;  // last never evicted
}

int SquisheState::find_min_priority_idx() const {
    int    idx = -1;
    double mn  = 1e30;
    for (int k = 1; k < count - 1; ++k) {
        if (buf[k].priority < mn) {
            mn  = buf[k].priority;
            idx = k;
        }
    }
    return idx;
}

bool SquisheState::feed(const TradPoint &pt) {
    if (count < SQUISHE_BUF) {
        buf[count].pt       = pt;
        buf[count].priority = 0.0;
        buf[count].valid    = true;
        ++count;
        recalc_priorities();
        return false;
    }

    // Buffer full ¡ª evict the interior point with minimum SED priority
    int evict = find_min_priority_idx();
    if (evict < 0) {
        // No interior point (only 2 entries): just replace oldest
        evict = 0;
    }

    last_key = buf[evict].pt;
    has_key  = true;

    // Shift remaining points down
    for (int k = evict; k < count - 1; ++k)
        buf[k] = buf[k + 1];
    --count;

    // Insert new point at end
    buf[count].pt       = pt;
    buf[count].priority = 0.0;
    buf[count].valid    = true;
    ++count;

    recalc_priorities();
    return true;
}

bool SquisheState::flush(TradPoint &out) {
    if (count == 0) return false;
    out = buf[0].pt;
    // Shift
    for (int k = 0; k < count - 1; ++k) buf[k] = buf[k+1];
    --count;
    last_key = out;
    has_key  = true;
    return true;
}

// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T
//  4. VOLTCom
// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T

void VoltcomState::init(double eps_m) {
    epsilon = eps_m;
    has_seg = false;
    has_key = false;
    memset(&seg_start, 0, sizeof(seg_start));
    memset(&seg_end,   0, sizeof(seg_end));
    memset(&last_key,  0, sizeof(last_key));
}

bool VoltcomState::feed(const TradPoint &pt) {
    if (!has_seg) {
        seg_start = pt;
        seg_end   = pt;
        has_seg   = true;
        // First point is always a key-point
        last_key = pt;
        has_key  = true;
        return true;
    }

    // Estimate position at pt.ts using current vector seg_start ¡ú seg_end
    double dt_seg = static_cast<double>(
        static_cast<int64_t>(seg_end.ts) - static_cast<int64_t>(seg_start.ts));
    double est_lat, est_lon;
    if (dt_seg <= 0.0) {
        est_lat = seg_start.lat;
        est_lon = seg_start.lon;
    } else {
        double t = static_cast<double>(
            static_cast<int64_t>(pt.ts) - static_cast<int64_t>(seg_start.ts)) / dt_seg;
        est_lat = seg_start.lat + t * (seg_end.lat - seg_start.lat);
        est_lon = seg_start.lon + t * (seg_end.lon - seg_start.lon);
    }

    double err = haversine_m(pt.lat, pt.lon, est_lat, est_lon);

    if (err < epsilon) {
        // Extend current segment's endpoint
        seg_end = pt;
        return false;
    } else {
        // Close current segment: emit seg_end as key-point, start new segment
        last_key  = seg_end;
        has_key   = true;
        seg_start = seg_end;
        seg_end   = pt;
        return true;
    }
}

bool VoltcomState::flush() {
    if (!has_seg) return false;
    last_key = seg_end;
    has_key  = true;
    return true;
}

TradPoint VoltcomState::decompress_at(uint64_t ts) const {
    TradPoint out;
    out.ts = ts;
    double dt = static_cast<double>(
        static_cast<int64_t>(seg_end.ts) - static_cast<int64_t>(seg_start.ts)); 
    if (dt <= 0.0) {
        out.lat = seg_start.lat;
        out.lon = seg_start.lon;
        return out;
    }
    double t = static_cast<double>(
        static_cast<int64_t>(ts) - static_cast<int64_t>(seg_start.ts)) / dt;
    out.lat = seg_start.lat + t * (seg_end.lat - seg_start.lat);
    out.lon = seg_start.lon + t * (seg_end.lon - seg_start.lon);
    return out;
}
