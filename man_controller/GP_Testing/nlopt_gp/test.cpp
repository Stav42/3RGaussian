#include <iostream>
#include <Python.h>
#include <numpy/arrayobject.h>

int main() {
    Py_Initialize();
    if (_import_array() < 0) {
        PyErr_Print();
        return -1;
    }

    PyObject* pName = PyUnicode_DecodeFSDefault("gp_fit");
    PyObject* pModule = PyImport_Import(pName);
    Py_XDECREF(pName);

    if (pModule == NULL) {
        PyErr_Print();
        return -1;
    }

    PyObject* pFunc = PyObject_GetAttrString(pModule, "array");
    if (pFunc == NULL || !PyCallable_Check(pFunc)) {
        PyErr_Print();
        return -1;
    }

    PyObject* pValue = PyObject_CallObject(pFunc, NULL);
    if (pValue == NULL) {
        PyErr_Print();
        return -1;
    }

    PyArrayObject* npArray = (PyArrayObject*)pValue;
    int size = PyArray_SIZE(npArray);
    double* cArray = (double*)PyArray_DATA(npArray);

    for (int i = 0; i < size; ++i) {
        std::cout << cArray[i] << " ";
    }
    std::cout << std::endl;

    Py_XDECREF(pValue);
    Py_XDECREF(pFunc);
    Py_XDECREF(pModule);
    Py_Finalize();

    return 0;
}
