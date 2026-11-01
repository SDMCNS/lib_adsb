"""Wire format parsers and serializers."""

from __future__ import annotations

from adsb.wire.avr import parse_avr_frame, parse_avr_stream, serialize_avr_frame
from adsb.wire.beast import (
    BeastFrame,
    parse_beast_frame,
    parse_beast_stream,
    serialize_beast_frame,
)
from adsb.wire.json_codec import (
    parse_json_line,
    parse_json_stream,
    serialize_json_line,
    serialize_json_stream,
)
from adsb.wire.sbs import (
    SBSMessage,
    parse_sbs_message,
    parse_sbs_stream,
    serialize_sbs_message,
)

__all__ = [
    "BeastFrame",
    "SBSMessage",
    "parse_avr_frame",
    "parse_avr_stream",
    "parse_beast_frame",
    "parse_beast_stream",
    "parse_json_line",
    "parse_json_stream",
    "parse_sbs_message",
    "parse_sbs_stream",
    "serialize_avr_frame",
    "serialize_beast_frame",
    "serialize_json_line",
    "serialize_json_stream",
    "serialize_sbs_message",
]
