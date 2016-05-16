#!/usr/bin/env python3
import random
import numpy as np
import pandas as pd
import argparse
import sys


ARGPARSE_DESCRIPTION = "Script to parse Rossmann stores data"

NA_VALUE = ""

DATE_FORMAT = '%Y-%m-%d'

DTYPE_TEST = {
    'Id': np.int32,
    'Store': np.int32,
    'DayOfWeek': np.int8,
    'Open': np.object,          # there is some nan values
    'Promo': np.int8,
    'StateHoliday': np.object,  # categorical
    'SchoolHoliday': np.int8}

DTYPE_TRAIN = {
    'Id': np.int32,
    'Store': np.int32,
    'DayOfWeek': np.int8,
    'Sales': np.int32,
    'Customers': np.int32,
    'Open': np.int8,
    'Promo': np.int8,
    'StateHoliday': np.object,  # categorical
    'SchoolHoliday': np.int8}

DTYPE_STORE = {
    'Store': np.int32,
    'StoreType': np.object,
    'Assortment': np.object,
    'CompetitionDistance': np.float32,
    'CompetitionOpenSiceMonth': np.object,  # categorical
    'CompetitionOpenSiceYear': np.object,
    'Promo2': np.int8,
    'Promo2SinceWeek': np.object,
    'Promo2SinceYear': np.object,
    'PromoInterval': np.object
}


def load_data_file(filename, dtypes, parse_date=True, nrows=None):
    """
    Load file to data frame.
    """
    def date_parse(x): return pd.datetime.strptime(x, DATE_FORMAT)

    if parse_date:
        return pd.read_csv(filename, sep=',', parse_dates=['Date'],
                           date_parser=date_parse, dtype=dtypes, nrows=nrows,
                           na_values=NA_VALUE)
    else:
        return pd.read_csv(filename, sep=',', dtype=dtypes,
                           nrows=nrows, na_values=NA_VALUE)


def main(args):
    if args.type == "train":
        store_data = load_data_file(args.file, DTYPE_TRAIN, nrows=args.nrows)
    elif args.type == "test":
        store_data = load_data_file(args.file, DTYPE_TEST, nrows=args.nrows)
    elif args.type == "store":
        store_data = load_data_file(args.file, DTYPE_STORE, nrows=args.nrows, parse_date=False)
    else:
        print("Invalid database type")
        sys.exit(1)

    if args.sparse_mode is not None:
        if args.sparse_mode == "random":
            store_list = []
            random.seed()
            for i in range(10):
                store_list.append(random.randint(1, 1000))
        elif args.sparse_mode == "head":
            store_list = range(1, 11)
        else:
            store_list = range(990, 1001)

        if args.file_out:
            store_data_small = store_data.loc[store_data['Store'].isin(store_list)]
            store_data_small.to_csv(args.file_out, index=False, date_format=DATE_FORMAT)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=ARGPARSE_DESCRIPTION)
    parser.add_argument("type", choices=["train", "test", "store"],
                        help="Type of data base file")
    parser.add_argument("file", type=argparse.FileType('r'),
                        help="Name of database file")
    parser.add_argument("--sparse_mode", choices=["random", "head", "tail"],
                        help="mode for picking specific stores")
    parser.add_argument("--file_out", type=argparse.FileType('w'),
                        help="file to write data for selected stores")
    parser.add_argument("--nrows", type=int,
                        help="number of rows to read", default=None)
    args = parser.parse_args()

    main(args)