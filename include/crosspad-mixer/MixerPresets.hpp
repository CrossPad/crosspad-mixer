// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace crosspad {

/// Mixer presets: the whole desk -- every channel's level, mute, solo, pan and
/// sends, the outputs' levels, the inputs' gain -- saved under a name, one JSON
/// file a slot in the folder the platform names. A slot never saved offers a
/// starting point for what it is named after (STREAM, BEATS, SAMPLING); the
/// rest are empty until saved.
///
/// On a card, not in NVS: a desk is ~2 kB of JSON and the board's NVS is 24 kB
/// shared with every setting. Files also travel -- copied off the card, or
/// onto another board.
class MixerPresets {
public:
    static constexpr int    kSlots   = 6;
    static constexpr size_t kMaxName = 16;

    enum class Kind : uint8_t {
        Empty,      ///< nothing to load until something is saved here
        Factory,    ///< never saved: loads a starting point for its name
        Saved,      ///< a file on the card
    };
    struct Info {
        std::string name;
        Kind        kind = Kind::Empty;
    };

    /// The folder, set by the platform; "" = no presets here.
    void setDir(const std::string& dir) { dir_ = dir; }
    bool available() const { return !dir_.empty(); }
    const std::string& dir() const { return dir_; }

    /// Read every slot's name from the card: when the presets are shown, so
    /// opening the Mixer costs no card access.
    void refresh();
    const Info& info(int slot) const;

    /// The desk as it is now into @p slot, under @p name. False when the card
    /// cannot take it.
    bool save(int slot, const std::string& name);
    /// @p slot onto the mixer. The recording bus keeps its level -- that one
    /// is Settings' -- and the desk before it is kept for undo().
    bool load(int slot);
    /// The desk the last load replaced, back.
    bool undo();
    bool canUndo() const { return !undo_.empty(); }

private:
    std::string path(int slot) const;
    std::string snapshot(const std::string& name) const;
    bool        apply(const std::string& json);
    bool        applyFactory(int slot);

    std::string dir_;
    Info        info_[kSlots];
    std::string undo_;
};

MixerPresets& getMixerPresets();

} // namespace crosspad
