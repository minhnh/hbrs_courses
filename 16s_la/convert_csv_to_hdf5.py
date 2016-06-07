#!/usr/bin/env python3
"""
Code referenced from https://github.com/Cospel/kaggle-rossmann/
"""

import sys
import random
import argparse
import numpy as np
import pandas as pd
from pandas import HDFStore

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
    'Sales': np.float32,
    'Customers': np.float32,
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
    'StateHoliday' : {'a': 1, 'b': 2, 'c': 3, '0': 0},
    'Assortment' : {'a': 0, 'b': 1, 'c': 2},
    'StoreType' : {'a': 0, 'b': 1, 'c': 2, 'd': 3}
}


def date_parse(date_string):
    """:param date_string: date string from csv file """
    return pd.datetime.strptime(date_string, _DATE_FORMAT)


def load_data_file(filename, dtypes, parse_date=True, nrows=None):
    """
    Load file to data frame.

    :param filename: name of the file to be loaded
    :param parse_date:
    :param nrows:
    """
    if parse_date:
        return pd.read_csv(filename, sep=',', parse_dates=['Date'],
                           date_parser=date_parse, dtype=dtypes, nrows=nrows,
                           na_values=_NA_VALUE)
    else:
        return pd.read_csv(filename, sep=',', dtype=dtypes, nrows=nrows,
                           na_values=_NA_VALUE)


def normalize_data_by_column(data_train, data_store, column_name='Sales'):
    """
    :param data_train:
    :param data_store:
    :param column_name:
    """
    if 'Store' in data_store:
        stores = data_store['Store'].unique()
    else:
        stores = data_train['Store'].unique()
        data_store['Store'] = stores

    store_means = []
    store_means_openonly = []
    store_stds = []
    store_stds_openonly = []
    for store_id in data_store['Store']:
        store_train_data = data_train[column_name].loc[data_train['Store'] == store_id]
        store_train_data_openonly = data_train[column_name].loc[(data_train['Store'] == store_id) &
                                                                (data_train['Open'] == 1)]

        store_means.append(store_train_data.mean())
        store_means_openonly.append(store_train_data_openonly.mean())
        store_stds.append(store_train_data.std())
        store_stds_openonly.append(store_train_data_openonly.std())

        data_train.loc[data_train['Store'] == store_id, column_name + 'Norm'] \
            = (store_train_data - store_means[-1]) / store_stds[-1]

        data_train.loc[data_train['Store'] == store_id, column_name + "NormOpenOnly"] \
            = (store_train_data - store_means_openonly[-1]) / store_stds_openonly[-1]

    data_store['Mean' + column_name] = store_means
    data_store['Std' + column_name] = store_stds
    data_store['Mean' + column_name + 'OpenOnly'] = store_means_openonly
    data_store['Std' + column_name + 'OpenOnly'] = store_stds_openonly

    # add new columns to data_store
    data_store['Mean' + column_name] = store_means
    data_store['Std' + column_name] = store_stds

    return data_train, data_store


def process_data(train_data=None, test_data=None, store_data=None):
    """ Perform additional data processing """
    if train_data is not None:
        if store_data is None:
            store_data = pd.DataFrame()
        train_data = train_data.replace(_REPLACE_DICT)

    if test_data is not None:
        test_data = test_data.replace(_REPLACE_DICT)

    if store_data is not None:
        store_data = store_data.replace(_REPLACE_DICT)

    train_data, store_data = normalize_data_by_column(train_data, store_data,
                                                      column_name='Sales')
    train_data, store_data = normalize_data_by_column(train_data, store_data,
                                                      column_name='Customers')
    # Add WeekOfYear column
    train_data['WeekOfYear'] = train_data['Date'].map(lambda x: x.isocalendar()[1])
    test_data['WeekOfYear'] = test_data['Date'].map(lambda x: x.isocalendar()[1])

    return train_data, test_data, store_data


def main(args):
    """ main function """
    if args.num_store and args.num_store > _NUM_STORE_TOTAL:
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

    # Reverse dates
    if train_data is not None:
        train_data = train_data.iloc[::-1]
    if test_data is not None:
        test_data = test_data.iloc[::-1]

    if args.num_store:
        print("Selecting %d random stores" % (args.num_store))
        store_list = []
        random.seed()
        for _ in range(args.num_store):
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
    parser.add_argument("--num_store", type=int,
                        help="number of stores to select")
    parser.add_argument("file_out", type=argparse.FileType('w'),
                        help="file to write data for selected stores")
    parser.add_argument("--nrows", type=int,
                        help="number of rows to read", default=None)
    arguments = parser.parse_args()

    main(arguments)
