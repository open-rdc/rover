#!/usr/bin/env python3
"""Convert longitude/latitude coordinates stored in a CSV file into UTM."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Iterable

from pyproj import CRS, Transformer


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read longitude/latitude coordinates from a CSV file, convert them to UTM "
            "eastings/northings, and write a new CSV."
        )
    )
    parser.add_argument("input_csv", type=Path, help="Source CSV with lon/lat coordinates.")
    parser.add_argument(
        "output_csv",
        type=Path,
        nargs="?",
        help="Destination CSV path. Defaults to <cwd>/<input>_utm.csv.",
    )
    zone_group = parser.add_mutually_exclusive_group(required=False)
    zone_group.add_argument(
        "--zone",
        type=int,
        help="UTM zone number (1-60). Required unless --epsg is provided.",
    )
    zone_group.add_argument(
        "--epsg",
        type=int,
        help=(
            "EPSG code that defines the output coordinate reference system. "
            "Overrides --zone/--south/--datum."
        ),
    )
    parser.add_argument(
        "--south",
        action="store_true",
        help="Set if the desired UTM coordinates are in the southern hemisphere.",
    )
    parser.add_argument(
        "--datum",
        default="WGS84",
        help="Datum for the output CRS when using --zone (default: WGS84).",
    )
    parser.add_argument(
        "--lon-field",
        default="lon",
        help="Column name for the longitude value (default: lon).",
    )
    parser.add_argument(
        "--lat-field",
        default="lat",
        help="Column name for the latitude value (default: lat).",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Allow overwriting an existing output CSV file.",
    )
    parser.add_argument(
        "--precision",
        type=int,
        default=3,
        help="Number of decimal places for the output coordinates (default: 10).",
    )
    args = parser.parse_args()

    if args.epsg is None and args.zone is None:
        parser.error("Either --zone or --epsg must be provided to define the output CRS.")
    if args.precision < 0:
        parser.error("--precision must be zero or a positive integer.")

    return args


def resolve_output_path(input_csv: Path, output_csv: Path | None, overwrite: bool) -> Path:
    if output_csv is None:
        output_csv = Path.cwd() / f"{input_csv.stem}_utm.csv"
    if output_csv.exists() and not overwrite:
        raise FileExistsError(
            f"Output file {output_csv} already exists. Use --overwrite to replace it."
        )
    return output_csv


def build_transformer(zone: int | None, south: bool, datum: str, epsg: int | None) -> Transformer:
    wgs84 = CRS.from_epsg(4326)
    if epsg is not None:
        target_crs = CRS.from_epsg(epsg)
    else:
        if not 1 <= zone <= 60:
            raise ValueError(f"UTM zone must be in [1, 60], got {zone}.")
        proj_parts = ["+proj=utm", f"+zone={zone}", "+units=m", "+no_defs"]
        if datum:
            proj_parts.append(f"+datum={datum}")
        if south:
            proj_parts.append("+south")
        target_crs = CRS.from_proj4(" ".join(proj_parts))
    return Transformer.from_crs(wgs84, target_crs, always_xy=True)


def convert_rows(
    rows: Iterable[dict[str, str]],
    transformer: Transformer,
    lon_field: str,
    lat_field: str,
    precision: int,
) -> Iterable[dict[str, str]]:
    for row in rows:
        try:
            lon = float(row[lon_field])
            lat = float(row[lat_field])
        except KeyError as exc:
            raise KeyError(
                f"Missing required column: {exc.args[0]}"
            ) from exc
        except ValueError as exc:
            raise ValueError(
                f"Could not convert values to float for row {row}."
            ) from exc
        easting, northing = transformer.transform(lon, lat)
        formatted_x = f"{easting:.{precision}f}"
        formatted_y = f"{northing:.{precision}f}"
        yield {"x": formatted_x, "y": formatted_y}


def main() -> None:
    args = parse_arguments()
    output_path = resolve_output_path(args.input_csv, args.output_csv, args.overwrite)
    transformer = build_transformer(args.zone, args.south, args.datum, args.epsg)

    with args.input_csv.open(newline="", encoding="utf-8") as src:
        reader = csv.DictReader(src)
        if reader.fieldnames is None:
            raise ValueError("Input CSV is missing a header row.")
        rows = list(
            convert_rows(
                reader,
                transformer,
                args.lon_field,
                args.lat_field,
                args.precision,
            )
        )

    with output_path.open("w", newline="", encoding="utf-8") as dst:
        writer = csv.DictWriter(dst, fieldnames=["x", "y"])
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
