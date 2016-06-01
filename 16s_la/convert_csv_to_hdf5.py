#!/usr/bin/env python3
import random
import numpy as np
import pandas as pd
from pandas import HDFStore
import argparse
import sys

"""
Code referenced from https://github.com/Cospel/kaggle-rossmann/
"""

_ARGPARSE_DESCRIPTION = "Script to parse Rossmann stores data"

_NA_VALUE = ""

_DATE_FORMAT = '%Y-%m-%d'

_NUM_STORE_TOTAL = 1115

_ARG_TRAIN = "train"
_ARG_TEST = "test"
_ARG_STORE = "store"

_DTYPE_TEST = {
    'Id': np.int32,
    'Store': np.int32,
    'DayOfWeek': np.int8,
    'Open': np.object,          # there is some nan values
    'Promo': np.int8,
    'StateHoliday': np.object,  # categorical
    'SchoolHoliday': np.int8}

_DTYPE_TRAIN = {
    'Id': np.int32,
    'Store': np.int32,
    'DayOfWeek': np.int8,
    'Sales': np.int32,
    'Customers': np.int32,
    'Open': np.int8,
    'Promo': np.int8,
    'StateHoliday': np.object,  # categorical
    'SchoolHoliday': np.int8}

_DTYPE_STORE = {
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

_REPLACE_DICT = {
    'StateHoliday' : {'a': 1, 'b': 2, 'c': 3, '0': 0, 0: 0},
    'Assortment' : {'a': 0, 'b': 1, 'c': 2},
    'StoreType' : {'a': 0,'b': 1,'c': 2, 'd': 3}
}


def load_data_file(filename, dtypes, parse_date=True, nrows=None):
    """
    Load file to data frame.

    :param filename: name of the file to be loaded
    :param parse_date:
    :param nrows:
    """
    def date_parse(x): return pd.datetime.strptime(x, _DATE_FORMAT)

    if parse_date:
        return pd.read_csv(filename, sep=',', parse_dates=['Date'],
                           date_parser=date_parse, dtype=dtypes, nrows=nrows,
                           na_values=_NA_VALUE)
    else:
        return pd.read_csv(filename, sep=',', dtype=dtypes,
                           nrows=nrows, na_values=_NA_VALUE)


def process_data(train_data=None, test_data=None, store_data=None):
    if train_data is not None:
        if store_data is None:
            store_data = pd.DataFrame()
        train_data = train_data.replace(_REPLACE_DICT)

    if test_data is not None:
        test_data = test_data.replace(_REPLACE_DICT)

    if store_data is not None:
        store_data = store_data.replace(_REPLACE_DICT)

    return train_data, test_data, store_data


def main(args):
    if args.store_num and args.store_num > _NUM_STORE_TOTAL:
        sys.exit(1)
        print("[FAIL] Data should only have %d stores!" % (_NUM_STORE_TOTAL))
    if args.nrows and args.nrows < _NUM_STORE_TOTAL:
        print("[FAIL] Should read more rows than than the number of stores!")
        sys.exit(1)

    train_data = None
    test_data = None
    store_data = None
    print("Loading data files")
    if args.type == _ARG_TRAIN:
        train_data = load_data_file(args.file[0], _DTYPE_TRAIN, nrows=args.nrows)
    elif args.type == _ARG_TEST:
        test_data = load_data_file(args.file[0], _DTYPE_TEST, nrows=args.nrows)
    elif args.type == _ARG_STORE:
        store_data = load_data_file(args.file[0], _DTYPE_STORE, nrows=args.nrows,
                                    parse_date=False)
    elif args.type == "all":
        if len(args.file) < 3:
            print("all option requires three input files")
            sys.exit(1)
        train_data = load_data_file(args.file[0], _DTYPE_TRAIN, nrows=args.nrows)
        test_data = load_data_file(args.file[1], _DTYPE_TEST, nrows=args.nrows)
        store_data = load_data_file(args.file[2], _DTYPE_STORE, nrows=args.nrows,
                                    parse_date=False)
    else:
        print("Invalid database type")
        sys.exit(1)

    if args.store_num:
        print("Selecting %d random stores" % (args.store_num))
        store_list = []
        random.seed()
        for i in range(args.store_num):
            store_list.append(random.randint(1, _NUM_STORE_TOTAL + 1))
        if train_data is not None:
            train_data = train_data.loc[train_data['Store'].isin(store_list)]
        if test_data is not None:
            test_data = test_data.loc[test_data['Store'].isin(store_list)]
        if store_data is not None:
            store_data = store_data.loc[store_data['Store'].isin(store_list)]

    print("Perform additional processing...")
    train_data, test_data, store_data = process_data(train_data, test_data, store_data)

    print("Writing to HDF5 type file %s" % (args.file_out.name))
    hdf = HDFStore(args.file_out.name, mode='w')
    if train_data is not None:
        hdf.put(_ARG_TRAIN + "_data", train_data, format='table', data_columns=True)
    if test_data is not None:
        hdf.put(_ARG_TEST + "_data", test_data, format='table', data_columns=True)
    if store_data is not None:
        hdf.put(_ARG_STORE + "_data", store_data, format='table', data_columns=True)

    return


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=_ARGPARSE_DESCRIPTION)
    parser.add_argument("type", choices=[_ARG_TRAIN, _ARG_TEST, _ARG_STORE, "all"],
                        help="Type of data base file. If \"all\" then expects \
                        train, test, store order")
    parser.add_argument("file", type=argparse.FileType('r'), nargs='+',
                        help="Name of database file")
    parser.add_argument("--store_num", type=int,
                        help="number of stores to select")
    parser.add_argument("file_out", type=argparse.FileType('w'),
                        help="file to write data for selected stores")
    parser.add_argument("--nrows", type=int,
                        help="number of rows to read", default=None)
    args = parser.parse_args()

    main(args)
