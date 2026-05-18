# oxideav-mpeg12video

A pure-Rust MPEG-1 Video / MPEG-2 Video codec for the
[oxideav](https://github.com/OxideAV/oxideav) framework.

## Status

**Orphan-rebuild scaffold (2026-05-18).** The prior implementation was
retired under the workspace
[clean-room policy](https://github.com/OxideAV/oxideav/blob/master/docs/IMPLEMENTOR_ROUND.md):
the VLC table modules acknowledged that their numerical entries were
reproduced from an external library's data tables — clean-room
provenance for the table values could not be defended. Master history
was fully erased per the Hat-3 cold-enforcement procedure.

The implementation will be re-built against the published MPEG video
specifications (ISO/IEC 11172-2 for MPEG-1 Video; ISO/IEC 13818-2 for
MPEG-2 Video) in a future clean-room round.

## License

MIT — see [LICENSE](./LICENSE).
